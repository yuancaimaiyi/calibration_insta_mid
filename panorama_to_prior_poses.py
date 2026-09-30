#!/usr/bin/env python3
"""
panorama_to_prior_poses.py
--------------------------
端到端：panorama.mp4 (Insta360 拼接) + traj_lidar.txt (Mid360/GLIM)
      → 每帧图像 + 每帧 LiDAR 先验位姿 (prior_poses.txt)

统一时轴：**绝对 host 秒** (1.78e9 量级)
  - traj_lidar.txt 第一列本就是 host 秒 → 直接用
  - 视频第 i 帧的 host 秒 = record_start_host_ns/1e9 + PTS_i + offset_sec

外参（未标定）：仿真 T_lidar_camera (Insta360 相对 Mid360)
时间偏置：sync_offset.txt 的 offset_sec；本次按需求 mock = 0.0

参考实现：/home/yuancaimaiyi/qiudao_0512/sync/assign_poses.py
时间基说明：/media/yuancaimaiyi/yuancaimaiyi/data/TIME_SYNC_AND_FRAME_TIMESTAMP.md

用法（默认参数即可跑通当前数据）：
    python3 panorama_to_prior_poses.py
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation, Slerp


# ─── 默认路径 ────────────────────────────────────────────────────────────────
DATA_DIR         = Path("/media/yuancaimaiyi/yuancaimaiyi/data")
OUT_DIR          = DATA_DIR / "prior_pose_pipeline_20260914101939"

DEFAULT_SESSION  = DATA_DIR / "20260914101939_Lioos_office.session.json"
DEFAULT_PANORAMA = Path("/home/yuancaimaiyi/xreal_metaloc/hera-output/"
                        "25618842-2431-4b9c-ae7d-79134c89ae36/step_stitch/panorama.mp4")
DEFAULT_TRAJ     = Path("/home/yuancaimaiyi/xreal_metaloc/hera-output/"
                        "b4edd601-dbed-4e8e-9cdc-40b494185eff/step_recon/map/traj_lidar.txt")
DEFAULT_OFFSET   = DATA_DIR / "20260914101939_Lioos_office_sync_offset.txt"


# ═════════════════════════════════════════════════════════════════════════════
# I/O helpers
# ═════════════════════════════════════════════════════════════════════════════

def load_record_start_ns(session_json: Path) -> tuple[int, str]:
    """
    视频首帧的 host 时间锚点，按可靠性优先级：
      1. frame_0_host_ns —— 由 patch_session_with_gyro_bridge.py 用 gyro clock bridge
         (camera↔host) 算出的第 0 帧 host 时刻，精度 ~1-5 ms，最靠谱
      2. first_gps_timestamp_ns —— 由 patch_session_with_gps.py 从 .insv metadata
         提取的相机 RTC (曾被 GPS/手机 App 校准) 时戳，精度 ~10-100 ms，看 RTC 漂移
      3. record_start_host_ns —— host 端"用户点采集 button"那一刻（SDK 异步
         立刻返回）。相机曝光第 0 帧比它晚几百 ms 到上千 ms（USB + camera
         startup），每次开机不同。仅作 fallback

    返回 (host_ns, source_label)。
    """
    with open(session_json) as f:
        d = json.load(f)
    bridge = int(d.get("frame_0_host_ns", 0))
    if bridge > 0:
        return bridge, "frame_0_host_ns (gyro clock bridge, ~1-5 ms accurate)"
    gps = int(d.get("first_gps_timestamp_ns", 0))
    if gps > 0:
        return gps, "first_gps_timestamp_ns (insv RTC anchor, ~10-100 ms accurate)"
    return int(d["record_start_host_ns"]), "record_start_host_ns (SDK-return time, biased by SDK sync delay ~hundreds of ms)"


def load_offset_sec(offset_file: Path, force_zero: bool) -> tuple[float, str]:
    """
    读取 sync_offset.txt。语义：t_mid = t_insta + offset_sec。
    - force_zero=True         → 强制 0（调试用）
    - 文件不存在              → 0（打警告）
    - 文件存在但无 offset_sec → 0（打警告）
    - 文件存在且有值          → 使用该值

    **重要**：此处一旦决定了 offset_sec，后续帧命名的 <host_ns>.jpg 就把它固化进
    文件名。calibrate_extrinsic 直接从文件名反解 host_sec 对齐 traj_lidar，
    不再需要 offset 参与。offset 在整个 pipeline 里只在这里进入系统一次。
    """
    if force_zero:
        return 0.0, "forced zero (--mock_offset_zero)"
    if not offset_file.exists():
        return 0.0, f"file missing: {offset_file}"
    with open(offset_file) as f:
        for line in f:
            line = line.strip()
            if line.startswith("offset_sec="):
                return float(line.split("=", 1)[1]), f"loaded from {offset_file.name}"
    return 0.0, f"no `offset_sec=` in {offset_file}"


def probe_video_pts(mp4: Path) -> list[float]:
    """抽取每帧 PTS (秒)。优先 pts_time，缺失时回落到 best_effort_timestamp_time。"""
    raw = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v",
        "-show_entries", "frame=pts_time,best_effort_timestamp_time",
        "-of", "csv=p=0", str(mp4),
    ]).decode()
    out = []
    for line in raw.splitlines():
        if not line.strip():
            continue
        parts = [p for p in line.split(",") if p.strip()]
        if not parts:
            continue
        out.append(float(parts[0]))
    return out


def load_lidar_traj_abs_sec(path: Path):
    """
    读取 GLIM traj_lidar.txt
      格式：t_sec  tx ty tz  qx qy qz qw
      时间基：host wall clock 绝对秒 (~1.78e9)
    不做归零 — 与视频帧的 host_sec 直接对齐。
    """
    data = np.loadtxt(path, comments="#")
    if data.ndim == 1:
        data = data[np.newaxis, :]
    order = np.argsort(data[:, 0])
    data = data[order]
    ts    = data[:, 0]
    trans = data[:, 1:4]
    quats = data[:, 4:8]        # qx qy qz qw
    return ts, trans, quats


# ═════════════════════════════════════════════════════════════════════════════
# 外参仿真：T_lidar_camera (Mid360 → Insta360 光心)
# ═════════════════════════════════════════════════════════════════════════════

def simulate_extrinsic(out_path: Path) -> np.ndarray:
    """
    T_lidar_camera (camera → lidar)：Insta360 光心在 Mid360 前方 5cm、上方 8cm，
    姿态基本一致，仅 2° 俯仰模拟安装偏差。仅用于占位，标定后请用 --extrinsic_file 覆盖。
    """
    t = np.array([0.05, 0.0, 0.08])
    r = Rotation.from_euler("xyz", [0.0, 2.0, 0.0], degrees=True)
    q = r.as_quat()                                 # qx qy qz qw
    T = np.eye(4); T[:3, :3] = r.as_matrix(); T[:3, 3] = t

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(out_path, np.concatenate([t, q])[None, :],
               header="tx ty tz qx qy qz qw  (simulated T_lidar_camera, camera->lidar)",
               comments="# ", fmt="%.9f")
    return T


def load_extrinsic(path: Path) -> np.ndarray:
    """
    读取 T_lidar_camera (camera → lidar)。
    格式：单行 `tx ty tz qx qy qz qw`，允许 `#` 注释头。
    """
    data = np.loadtxt(path, comments="#").ravel()
    if data.size != 7:
        raise ValueError(f"extrinsic file {path} expects 7 numbers (tx ty tz qx qy qz qw), "
                         f"got {data.size}")
    t, q = data[:3], data[3:7]
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3,  3] = t
    return T


# ═════════════════════════════════════════════════════════════════════════════
# 抽帧 + 命名
# ═════════════════════════════════════════════════════════════════════════════

def extract_and_rename_frames(mp4: Path,
                              pts: list[float],
                              record_start_ns: int,
                              offset_sec: float,
                              traj_t0: float,
                              traj_t1: float,
                              frames_dir: Path,
                              csv_out: Path,
                              target_fps: float | None = None,
                              max_frames: int | None = None) -> int:
    """
    1) ffmpeg 抽全部帧到 frames_dir/_tmp/<8-digit>.jpg
    2) 按 host_ns 重命名，仅保留落在 [traj_t0, traj_t1] 内的帧
    3) 可选下采样：
         · target_fps: 相邻保留帧 host_sec 间隔 >= 1/fps
         · max_frames: 等间隔取样至不超过 N 帧（若同时给 target_fps，取二者更严）
    4) 写 frame_timestamps.csv：frame_index, image_name, t_host_sec
    """
    frames_dir.mkdir(parents=True, exist_ok=True)
    tmp = frames_dir / "_tmp"
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir()

    print(f"  [ffmpeg] extracting frames → {tmp}/ …")
    subprocess.check_call([
        "ffmpeg", "-loglevel", "error", "-y", "-i", str(mp4),
        "-vsync", "0", "-qscale:v", "2", str(tmp / "%08d.jpg"),
    ])

    # 先算出落在轨迹窗内的候选帧列表（i, host_sec）
    in_window = []
    for i, p in enumerate(pts):
        host_sec = record_start_ns / 1e9 + p + offset_sec
        if traj_t0 <= host_sec <= traj_t1:
            in_window.append((i, host_sec))

    # max_frames → 转成等效的最小间隔
    min_dt = 0.0
    if target_fps is not None and target_fps > 0:
        min_dt = 1.0 / target_fps
    if max_frames is not None and max_frames > 0 and len(in_window) > max_frames:
        span = in_window[-1][1] - in_window[0][1]
        min_dt = max(min_dt, span / max_frames)

    keep_flags = [False] * len(in_window)
    if min_dt > 0:
        last_t = -np.inf
        for k, (_, t) in enumerate(in_window):
            if t - last_t >= min_dt:
                keep_flags[k] = True
                last_t = t
    else:
        keep_flags = [True] * len(in_window)

    keep_set = {in_window[k][0] for k, keep in enumerate(keep_flags) if keep}

    kept = 0
    with open(csv_out, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["frame_index", "image_name", "t_host_sec"])

        for i, p in enumerate(pts):
            src = tmp / f"{i + 1:08d}.jpg"                # ffmpeg 从 00000001 起
            if i not in keep_set:
                src.unlink(missing_ok=True)
                continue
            host_sec = record_start_ns / 1e9 + p + offset_sec
            host_ns  = int(round(host_sec * 1e9))
            dst_name = f"{host_ns}.jpg"
            src.rename(frames_dir / dst_name)
            w.writerow([i, dst_name, f"{host_sec:.9f}"])
            kept += 1

    shutil.rmtree(tmp, ignore_errors=True)
    if min_dt > 0:
        print(f"  [subsample] target_fps={target_fps} max_frames={max_frames} "
              f"→ min_dt={min_dt*1000:.1f}ms, kept {kept}/{len(in_window)} in-window")
    return kept


# ═════════════════════════════════════════════════════════════════════════════
# 位姿赋值
# ═════════════════════════════════════════════════════════════════════════════

def assign_poses(frames_csv: Path,
                 ts_mid: np.ndarray,
                 trans_mid: np.ndarray,
                 quats_mid: np.ndarray,
                 T_lc: np.ndarray,
                 out_path: Path):
    """
    对每帧：
      T_world_camera = T_world_lidar(t_host_sec) @ T_lidar_camera
    输出 TUM 风格：image_name tx ty tz qx qy qz qw
    """
    trans_interp = interp1d(ts_mid, trans_mid, axis=0,
                            bounds_error=False,
                            fill_value=(trans_mid[0], trans_mid[-1]))
    slerp = Slerp(ts_mid, Rotation.from_quat(quats_mid))

    written = skipped = 0
    with open(frames_csv) as fp, open(out_path, "w") as out:
        out.write("# image_name tx ty tz qx qy qz qw\n")
        out.write("# T_world_camera = T_world_lidar(t_host) @ T_lidar_camera\n")
        out.write("# time_base = absolute host seconds (aligned with traj_lidar.txt)\n")

        for row in csv.DictReader(fp):
            t = float(row["t_host_sec"])
            if not (ts_mid[0] <= t <= ts_mid[-1]):
                skipped += 1
                continue
            t_wl = trans_interp(t)
            q_wl = slerp([float(np.clip(t, ts_mid[0], ts_mid[-1]))]).as_quat()[0]
            T_wl = np.eye(4)
            T_wl[:3, :3] = Rotation.from_quat(q_wl).as_matrix()
            T_wl[:3,  3] = t_wl

            T_wc = T_wl @ T_lc
            tc   = T_wc[:3, 3]
            qc   = Rotation.from_matrix(T_wc[:3, :3]).as_quat()
            out.write(f"{row['image_name']} "
                      f"{tc[0]:.6f} {tc[1]:.6f} {tc[2]:.6f} "
                      f"{qc[0]:.6f} {qc[1]:.6f} {qc[2]:.6f} {qc[3]:.6f}\n")
            written += 1

    return written, skipped


# ═════════════════════════════════════════════════════════════════════════════
# 主流程
# ═════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--panorama",  type=Path, default=DEFAULT_PANORAMA)
    ap.add_argument("--session",   type=Path, default=DEFAULT_SESSION)
    ap.add_argument("--traj",      type=Path, default=DEFAULT_TRAJ)
    ap.add_argument("--offset",    type=Path, default=DEFAULT_OFFSET,
                    help="sync_offset.txt 路径（含 offset_sec=<value>）")
    ap.add_argument("--mock_offset_zero", action="store_true", default=False,
                    help="强制 offset_sec = 0（调试用）。默认关闭：优先读 --offset 文件，"
                         "读不到才回落 0 并给警告。")
    ap.add_argument("--extrinsic_file", type=Path, default=None,
                    help="T_lidar_camera (camera→lidar)：单行 tx ty tz qx qy qz qw。"
                         "缺省时仿真一个占位外参并写入 out_dir/extrinsic_sim.txt")
    ap.add_argument("--out_dir",   type=Path, default=OUT_DIR)
    ap.add_argument("--target_fps", type=float, default=None,
                    help="下采样到目标帧率（如 3.0）。基于 host_sec 时间等间隔挑帧，"
                         "保证相邻保留帧间隔 >= 1/fps。默认不下采样。")
    ap.add_argument("--max_frames", type=int, default=None,
                    help="下采样到不超过 N 帧。可与 --target_fps 同用（取更严）。")
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    frames_dir  = args.out_dir / "frames"
    frames_csv  = args.out_dir / "frame_timestamps.csv"
    extr_file   = args.out_dir / "extrinsic_sim.txt"
    poses_file  = args.out_dir / "prior_poses.txt"

    print(f"[cfg] panorama = {args.panorama}")
    print(f"[cfg] session  = {args.session}")
    print(f"[cfg] traj     = {args.traj}")
    print(f"[cfg] offset   = {args.offset}  (mock_zero={args.mock_offset_zero})")
    print(f"[cfg] out_dir  = {args.out_dir}")

    # ── 1. 时基锚点 & offset ────────────────────────────────────────────────
    record_start_ns, anchor_src = load_record_start_ns(args.session)
    offset_sec, offset_src = load_offset_sec(args.offset, args.mock_offset_zero)
    print(f"[t0 ] video anchor host_ns  = {record_start_ns}  ({anchor_src})")
    print(f"[t0 ] offset_sec            = {offset_sec:.6f} s  ({offset_src})")
    if "SDK-return" in anchor_src:
        print(f"[warn] session.json 缺 first_gps_timestamp_ns，回落到 record_start_host_ns。"
              f"抽帧时间戳会有百毫秒级系统偏（走路 1 m/s → 10 cm+ 位置误差）。"
              f"运行 patch_session_with_gps.py 从 .insv 提取 GPS 时戳后重跑。")
    if offset_sec == 0.0 and not args.mock_offset_zero:
        print(f"[warn] offset_sec = 0：帧命名会有 pipeline 延迟差（数十~数百 ms）的系统偏。"
              f"下游 calibrate_extrinsic 的 T_lc 会带同量级方向偏差。sync 有解后请重跑。")

    # ── 2. 轨迹（绝对 host 秒） ─────────────────────────────────────────────
    print(f"[traj] loading …")
    ts_mid, trans_mid, quats_mid = load_lidar_traj_abs_sec(args.traj)
    traj_t0, traj_t1 = float(ts_mid[0]), float(ts_mid[-1])
    print(f"[traj] {len(ts_mid)} poses, host_sec ∈ [{traj_t0:.3f}, {traj_t1:.3f}] "
          f"(span {traj_t1 - traj_t0:.2f}s)")

    # ── 3. 抽 PTS & 覆盖分析 ───────────────────────────────────────────────
    print(f"[pts ] probing panorama.mp4 …")
    pts = probe_video_pts(args.panorama)
    v_t0 = record_start_ns / 1e9 + pts[0]  + offset_sec
    v_t1 = record_start_ns / 1e9 + pts[-1] + offset_sec
    print(f"[pts ] {len(pts)} frames, host_sec ∈ [{v_t0:.3f}, {v_t1:.3f}] "
          f"(span {v_t1 - v_t0:.2f}s)")
    ov_lo, ov_hi = max(v_t0, traj_t0), min(v_t1, traj_t1)
    print(f"[pts ] overlap window       = [{ov_lo:.3f}, {ov_hi:.3f}] "
          f"({max(0.0, ov_hi - ov_lo):.2f}s)")
    if ov_hi <= ov_lo:
        sys.exit("[error] 视频与轨迹时间轴无重叠，请检查 record_start 与 traj 时基")

    # ── 4. 抽帧并按 host_ns 命名 ───────────────────────────────────────────
    print(f"[img ] extracting + renaming …")
    kept = extract_and_rename_frames(args.panorama, pts, record_start_ns,
                                     offset_sec, traj_t0, traj_t1,
                                     frames_dir, frames_csv,
                                     target_fps=args.target_fps,
                                     max_frames=args.max_frames)
    print(f"[img ] kept {kept}/{len(pts)} frames → {frames_dir}/")

    # ── 5. 外参 T_lidar_camera (camera → lidar) ────────────────────────────
    if args.extrinsic_file is not None:
        print(f"[extr] loading T_lidar_camera ← {args.extrinsic_file}")
        T_lc = load_extrinsic(args.extrinsic_file)
    else:
        print(f"[extr] simulating T_lidar_camera → {extr_file}")
        T_lc = simulate_extrinsic(extr_file)
    print(f"[extr] translation (camera origin in lidar) = {T_lc[:3, 3]}")

    # ── 6. 位姿赋值 ────────────────────────────────────────────────────────
    print(f"[pose] assigning …")
    written, skipped = assign_poses(frames_csv, ts_mid, trans_mid, quats_mid,
                                    T_lc, poses_file)
    print(f"[pose] wrote {written}, skipped {skipped} → {poses_file}")

    # ── 7. 预览 ───────────────────────────────────────────────────────────
    print("\n---- prior_poses.txt (head) ----")
    with open(poses_file) as f:
        for i, line in enumerate(f):
            if i >= 8:
                break
            print(line.rstrip())


if __name__ == "__main__":
    main()
