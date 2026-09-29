#!/usr/bin/env python3
"""
patch_session_with_gyro_bridge.py
---------------------------------
用 gyro clock bridge 计算视频第 0 帧的 host 时间戳，patch 到 session.json。

原理：
  相机内钟同时驱动 gyro 采样和视频曝光；host 端 gyro callback 有 host_ts。
  每包 gyro 都携带 (host_ts, camera_ts) 双时戳，多包求 median →
  camera↔host clock offset。
  视频帧 0 的 host 时刻 = offset + first_frame_timestamp × μs→ns

单位陷阱（Insta360 X4 fw v1.9.21_build5 实测）：
  - .hera gyro sample.timestamp 单位 = ms  → 乘 1e6 转 ns
  - .insv first_frame_timestamp 单位 = μs  → 乘 1e3 转 ns
  同一相机内钟，两个字段单位不同！

精度：受 gyro callback 抖动限制，~1-5 ms（远优于 first_gps_timestamp 的
RTC 依赖 10-100ms，和 record_start_host_ns 的几百 ms - 秒偏差）。

依赖：
  - hera-storage-extract-insta（recorder 分支 feat/extract-insta-packet-host-ts
    之后的版本，CSV 含 packet_host_ns + timestamp_ms 两列）
  - telemetry-parser (python) 用于读 .insv 里 first_frame_timestamp

用法：
    python3 patch_session_with_gyro_bridge.py \\
        --hera     /path/to/xxx.hera \\
        --insv     /path/to/xxx.insv \\
        --session  /path/to/xxx.session.json \\
        --extract_insta /path/to/extract_insta   # 可省，默认从 recorder build 找
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def find_extract_insta(user_path: Path | None) -> str:
    """
    返回可调用的 extract 工具（绝对路径或命令名）。
    recorder 安装后 PATH 里应有 `hera-storage-extract-insta`
    （CMake target 名，来自 storage/CMakeLists.txt L101）。
    """
    if user_path and user_path.exists():
        return str(user_path)
    for name in ("hera-storage-extract-insta", "hera-extract-insta", "extract_insta"):
        which = shutil.which(name)
        if which:
            return which
    sys.exit("[error] 找不到 hera-storage-extract-insta；用 --extract_insta 指路径，"
             "或安装 recorder 让此命令进 PATH（apt install / cmake --install）")


def extract_gyro_csv(extract_bin: str, hera_path: Path) -> Path:
    """
    跑 hera-storage-extract-insta 输出 gyro CSV。
    CLI（对照 storage/tool/extract_insta.cpp main）:
        hera-storage-extract-insta <input.hera> --gyro <output.csv>
    """
    tmp = tempfile.NamedTemporaryFile(suffix=".csv", delete=False)
    tmp.close()
    csv_path = Path(tmp.name)
    print(f"[extract] {extract_bin} {hera_path} --gyro {csv_path}")
    subprocess.check_call([extract_bin, str(hera_path), "--gyro", str(csv_path)])
    return csv_path


def detect_camera_clock_unit(sample_diffs: list[int]) -> tuple[int, str]:
    """
    从相邻 gyro sample 时间差反推相机时钟单位。
    假设 IMU 采样率在 100-1000 Hz（周期 1-10 ms），单位可能是 ms/μs/ns。
    返回 (cam_to_ns 乘数, unit 字符串)。
    """
    import numpy as np
    med = float(np.median(sample_diffs))
    # ms 场景：1-10 ms/sample → raw diff 1-10
    if 0.5 <= med <= 20:
        return 1_000_000, "ms"
    # μs 场景：raw diff 1000-20000
    if 500 <= med <= 20_000:
        return 1_000, "μs"
    # ns 场景：raw diff 500_000-20_000_000
    if 500_000 <= med <= 20_000_000:
        return 1, "ns"
    sys.exit(f"[error] 无法从相邻差 median={med} 反推相机时钟单位；"
             f"人工检查 CSV 前几行")


def compute_clock_offset_ns(csv_path: Path) -> tuple[int, dict]:
    """
    从 gyro CSV 算 camera→host clock offset。
    - 每行 (packet_host_ns, camera_ts, ...)
    - 一个 packet 里多个 sample 共用一个 packet_host_ns。用每包最后一个 sample
      的 camera_ts 最接近 packet 到达时刻，做 offset = host - camera_ts。
    - 全量 median，抗抖动。
    返回 (offset_ns, diagnostics)。diagnostics 里含:
      - cam_to_ns 乘数
      - offset_median_ns / offset_std_ms / p10/p90
      - all_offsets_ns (完整数组, 给 --diag / --plot 用)
      - all_host_ns    (每包 host_ns, 用于看时间趋势)
    """
    import numpy as np

    pairs = []      # (packet_host_ns, last_camera_ts_in_that_packet)
    sample_diffs = []
    current_packet = None
    current_last_cam = None
    prev_cam = None
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            ph = int(row["packet_host_ns"])
            cam = int(row["timestamp_ms"])   # 相机内钟, ms（extract_insta CSV 语义）
            if ph != current_packet:
                if current_packet is not None:
                    pairs.append((current_packet, current_last_cam))
                current_packet = ph
            else:
                # 同一 packet 内的相邻 sample 差 → 反推单位
                if prev_cam is not None:
                    d = cam - prev_cam
                    if 0 < d < 1e9:   # 剔异常
                        sample_diffs.append(d)
            current_last_cam = cam
            prev_cam = cam
    if current_packet is not None:
        pairs.append((current_packet, current_last_cam))

    if len(pairs) < 5:
        sys.exit(f"[error] gyro packet 太少 ({len(pairs)})，无法算 clock bridge")
    if len(sample_diffs) < 5:
        sys.exit(f"[error] 无法从 CSV 反推相机时钟单位（同包 sample 太少）")

    cam_to_ns, unit = detect_camera_clock_unit(sample_diffs)

    offsets = np.array([p[0] - p[1] * cam_to_ns for p in pairs], dtype=np.int64)
    hosts   = np.array([p[0] for p in pairs], dtype=np.int64)
    med = int(np.median(offsets))
    std_ms = float(np.std(offsets) / 1e6)
    p10_p90 = (int(np.percentile(offsets, 10)), int(np.percentile(offsets, 90)))

    diag = {
        "n_packets": len(pairs),
        "cam_clock_unit": unit,
        "cam_to_ns_multiplier": cam_to_ns,
        "sample_diff_median": float(np.median(sample_diffs)),
        "offset_median_ns": med,
        "offset_std_ms": std_ms,
        "offset_p10_p90_ns": p10_p90,
        # for diagnostics / plotting only, not serialized to session.json
        "_all_offsets_ns": offsets,
        "_all_host_ns": hosts,
    }
    return med, diag


def print_offset_histogram(offsets_ns, host_ns, bin_ms: float = 1.0):
    """
    ASCII 直方图 + 时间趋势拟合，输出到 stdout。
    offsets_ns: 每包 offset 数组
    host_ns:    每包对应 host_ns（用于时间趋势）
    bin_ms:     直方图 bin 宽度（ms）
    """
    import numpy as np

    med_off = int(np.median(offsets_ns))
    dev_ns  = (offsets_ns - med_off).astype(np.float64)
    dev_ms  = dev_ns / 1e6

    # 分布统计
    print(f"\n[hist] offset 分布（相对 median 的偏离, bin={bin_ms}ms）")
    print(f"  n_packets = {len(offsets_ns)}, median = {med_off}")
    print(f"  min      = {dev_ms.min():+.2f} ms")
    print(f"  P10      = {np.percentile(dev_ms, 10):+.2f} ms")
    print(f"  MAD      = {np.median(np.abs(dev_ms)):.2f} ms  (中位绝对偏差, 抗离群)")
    print(f"  std      = {dev_ms.std():.2f} ms")
    print(f"  P90      = {np.percentile(dev_ms, 90):+.2f} ms")
    print(f"  max      = {dev_ms.max():+.2f} ms")
    print(f"  peak-peak= {(dev_ms.max()-dev_ms.min()):.2f} ms")

    # ASCII 直方图
    lo = int(np.floor(dev_ms.min() / bin_ms))
    hi = int(np.ceil(dev_ms.max() / bin_ms))
    edges = np.arange(lo, hi + 1) * bin_ms
    hist, _ = np.histogram(dev_ms, bins=edges)
    max_bar = 40
    peak = hist.max() if hist.max() > 0 else 1
    print(f"\n[hist] ASCII histogram (max bar = {peak} packets):")
    for i, count in enumerate(hist):
        if count == 0:
            continue
        left = edges[i]
        bar = "█" * int(count * max_bar / peak)
        marker = " ← peak" if count == peak else ""
        print(f"  [{left:+6.1f}, {left+bin_ms:+6.1f}) ms  {count:>4d}  {bar}{marker}")

    # 时间趋势（去除大常数，保留精度）
    if len(host_ns) >= 2:
        rel_t_s = ((host_ns - host_ns[0]) / 1e9).astype(np.float64)
        slope_ns_per_s, _ = np.polyfit(rel_t_s, dev_ns, 1)
        slope_ms_per_s = slope_ns_per_s / 1e6
        total_drift_ms = slope_ms_per_s * rel_t_s[-1]
        print(f"\n[trend] 时间趋势（线性拟合）")
        print(f"  录制时长 = {rel_t_s[-1]:.1f} s")
        print(f"  斜率     = {slope_ms_per_s:+.4f} ms/s  ({slope_ms_per_s*1000:+.1f} ppm)")
        print(f"  总漂移   = {total_drift_ms:+.2f} ms  (整段录制)")
        if abs(slope_ms_per_s) < 0.05:
            print(f"  ✅ 漂移 < 0.05 ms/s，可忽略（消费晶振典型 20 ppm = 0.02 ms/s）")
        elif abs(slope_ms_per_s) < 0.5:
            print(f"  ✓ 漂移正常晶振量级")
        else:
            print(f"  ⚠ 漂移偏大，长录制可能需要分段 offset")


def save_offset_plot(offsets_ns, host_ns, out_path: Path):
    """输出 offset 分布 + 时间趋势的 PNG。需要 matplotlib。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"[warn] --plot 需要 matplotlib（pip install matplotlib），跳过")
        return
    import numpy as np

    med_off = int(np.median(offsets_ns))
    dev_ms  = ((offsets_ns - med_off) / 1e6).astype(np.float64)
    rel_t_s = ((host_ns - host_ns[0]) / 1e9).astype(np.float64)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # (1) 直方图
    ax1.hist(dev_ms, bins=40, edgecolor="black", alpha=0.7)
    ax1.axvline(0, color="red", linestyle="--", label=f"median = {med_off} ns")
    ax1.set_xlabel("offset − median (ms)")
    ax1.set_ylabel("packet count")
    ax1.set_title(f"offset distribution (n={len(offsets_ns)}, "
                  f"std={dev_ms.std():.2f}ms, MAD={np.median(np.abs(dev_ms)):.2f}ms)")
    ax1.legend()
    ax1.grid(alpha=0.3)

    # (2) 时间趋势
    slope, intercept = np.polyfit(rel_t_s, dev_ms, 1)
    ax2.scatter(rel_t_s, dev_ms, s=6, alpha=0.5, label="per-packet offset")
    ax2.plot(rel_t_s, slope * rel_t_s + intercept, "r-",
             label=f"linear fit: {slope:+.4f} ms/s")
    ax2.set_xlabel("recording time (s)")
    ax2.set_ylabel("offset − median (ms)")
    ax2.set_title(f"clock drift over time (total {slope*rel_t_s[-1]:+.2f} ms)")
    ax2.legend()
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    print(f"[plot] saved → {out_path}")


def read_first_frame_timestamp_raw(insv_path: Path) -> int:
    """从 .insv metadata 读 first_frame_timestamp 原始值（相机内钟，单位由 gyro 反推）。"""
    try:
        import telemetry_parser
    except ImportError:
        sys.exit("[error] pip install telemetry-parser 后重试")

    tp = telemetry_parser.Parser(str(insv_path))
    tel = tp.telemetry()
    md = None
    for item in tel:
        if isinstance(item, dict):
            d = item.get("Default", item)
            if isinstance(d, dict) and "Metadata" in d:
                md = d["Metadata"]; break
    if md is None or "first_frame_timestamp" not in md:
        sys.exit("[error] .insv 里没有 first_frame_timestamp")
    return int(md["first_frame_timestamp"])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hera",     type=Path, required=True)
    ap.add_argument("--insv",     type=Path, required=True)
    ap.add_argument("--session",  type=Path, required=True)
    ap.add_argument("--extract_insta", type=Path, default=None,
                    help="extract_insta 二进制路径")
    ap.add_argument("--fft_unit_multiplier", type=int, default=1000,
                    help="first_frame_timestamp raw → ns 的乘数。"
                         "1000=μs (Insta X4 fw v1.9+ 默认), "
                         "1_000_000=ms (老固件), 1=ns。"
                         "如果 delta>>预期，多半是单位错")
    ap.add_argument("--diag", action="store_true",
                    help="打印 offset 分布 ASCII 直方图 + 时间漂移诊断"
                         "（不影响写入结果）")
    ap.add_argument("--plot", type=Path, default=None,
                    help="额外输出 offset 分布 + 时间趋势 PNG 到指定路径"
                         "（需 matplotlib）")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    for p in (args.hera, args.insv, args.session):
        if not p.exists():
            sys.exit(f"[error] {p} 不存在")

    # 1) gyro CSV
    ext_bin = find_extract_insta(args.extract_insta)
    csv_path = extract_gyro_csv(ext_bin, args.hera)

    try:
        # 2) camera↔host offset（从 gyro 相邻差反推 gyro 侧单位，实测 = ms）
        offset_ns, diag = compute_clock_offset_ns(csv_path)
        print(f"\n[bridge] gyro packets    = {diag['n_packets']}")
        print(f"[bridge] gyro cam unit   = {diag['cam_clock_unit']}  "
              f"(sample_diff_median={diag['sample_diff_median']:.2f})")
        print(f"[bridge] offset median   = {offset_ns} ns")
        print(f"[bridge] offset  std     = {diag['offset_std_ms']:.2f} ms   ← 越小越好")
        print(f"[bridge] offset P10/P90  = {diag['offset_p10_p90_ns']}")
        if diag['offset_std_ms'] > 20:
            print(f"[warn] offset std > 20ms，clock bridge 可能不稳；检查 gyro 抖动或 packet loss")

        # 2.5) 可选：offset 分布诊断
        if args.diag or args.plot:
            print_offset_histogram(diag["_all_offsets_ns"], diag["_all_host_ns"])
        if args.plot:
            save_offset_plot(diag["_all_offsets_ns"], diag["_all_host_ns"], args.plot)

        # 3) .insv first_frame_timestamp（单位独立！Insta X4 fw v1.9+ = μs）
        #    ⚠ first_frame_timestamp 和 gyro sample.timestamp 都是"相机开机 uptime"
        #      但单位不同：gyro=ms、fft=μs（相差 1000 倍），需要独立乘数
        #    delta (frame_0 - record_start) 一般在 -几百 ms 到 -几秒 之间，
        #    每次开机/固件/USB 状态不同（不是常数，不要写死补偿）
        fft_cam_to_ns = args.fft_unit_multiplier   # default 1000 (μs)
        fft_raw = read_first_frame_timestamp_raw(args.insv)
        fft_ns = fft_raw * fft_cam_to_ns
        fft_unit_name = {1: "ns", 1000: "μs", 1_000_000: "ms"}.get(fft_cam_to_ns, f"×{fft_cam_to_ns}")
        print(f"\n[insv] first_frame_timestamp raw = {fft_raw} ({fft_unit_name})")
        print(f"[insv]                       (ns) = {fft_ns}")

        # 4) 拼锚点
        frame_0_host_ns = offset_ns + fft_ns
        print(f"\n[anchor] frame_0_host_ns = {frame_0_host_ns} ns")

        # 5) sanity check：|delta| 应该在几秒以内，符号可正可负：
        #    - delta > 0（frame_0 晚于 record_start）：SDK 异步返回，camera startup 慢
        #      Insta360 X4 就是这种，typical +200~+1500 ms
        #    - delta < 0（frame_0 早于 record_start）：SDK 阻塞等 ACK 再返回
        #      少见，且量级小 (-几十 ms)
        #    - |delta| > 5s：几乎肯定单位错
        sess = json.loads(args.session.read_text())
        rs = int(sess.get("record_start_host_ns", 0))
        delta_ms = (frame_0_host_ns - rs) / 1e6
        print(f"[chk] record_start_host_ns  = {rs}")
        print(f"[chk] frame_0 - record_start = {delta_ms:+.1f} ms")
        if abs(delta_ms) > 5000:
            print(f"[ERROR] |delta| > 5s，几乎肯定单位错")
            print(f"        → gyro CSV 相邻差 = {diag['sample_diff_median']:.2f},")
            print(f"           fft raw = {fft_raw}, 用了单位 {fft_unit_name}")
        elif abs(delta_ms) > 2000:
            print(f"[warn] |delta| > 2s，量级偏大，请人工核查（正常一般 < 1.5s）")

        if args.dry_run:
            print("\n[dry-run] 未写盘")
            return

        sess["frame_0_host_ns"] = frame_0_host_ns
        sess["_frame_0_source"] = f"gyro_bridge (offset_std={diag['offset_std_ms']:.2f}ms)"
        # 剔除完整数组（避免 session.json 巨大），只保留可序列化诊断
        diag_serializable = {k: v for k, v in diag.items() if not k.startswith("_")}
        sess["_gyro_bridge_diag"] = diag_serializable
        args.session.write_text(json.dumps(sess, indent=2, default=str) + "\n")
        print(f"\n[out] patched → {args.session}")
    finally:
        csv_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
