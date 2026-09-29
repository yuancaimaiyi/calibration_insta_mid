#!/usr/bin/env python3
"""
hera_to_rosbag.py
-----------------
把 hera-desktop 已转好的 lidar rosbag2 (.db3) + Insta360 panorama.mp4
合成一个新的 ROS2 bag，喂给 direct_visual_lidar_calibration 做外参标定。

设计思路：
  - 复用 hera-desktop 输出的 db3（已含 /lidar/mid360/point_cloud2 + /lidar/mid360/imu）
  - 从 panorama.mp4 抽帧，用 gyro-bridge 修正后的 frame_0_host_ns 打 host 时间戳
  - 每帧封成 sensor_msgs/msg/Image + sensor_msgs/msg/CameraInfo (equirectangular)
  - 全部塞进新 bag 一起交给 direct_visual_lidar_calibration

前提：
  - session.json 已跑过 patch_session_with_gyro_bridge.py (含 frame_0_host_ns)
  - hera-desktop 已把 .hera 转成 .db3 (含 /lidar/mid360/point_cloud2)
  - panorama.mp4 是从对应 .insv 拼出来的（hera-desktop stitcher）
  - pip install rosbags

产出可直接跑（Docker 内）:
  ros2 run direct_visual_lidar_calibration preprocess_ros2 \\
      <output_bag_parent_dir> <preprocessed_out> \\
      --camera_model equirectangular \\
      --image_topic /insta/image \\
      --points_topic /lidar/mid360/point_cloud2 \\
      --camera_info_topic /insta/camera_info
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path


def load_video_anchor_ns(session_json: Path) -> tuple[int, str]:
    """
    优先级 gyro_bridge > gps > record_start（跟 panorama_to_prior_poses.py 一致）。
    """
    d = json.loads(session_json.read_text())
    bridge = int(d.get("frame_0_host_ns", 0))
    if bridge > 0:
        return bridge, "frame_0_host_ns (gyro bridge)"
    gps = int(d.get("first_gps_timestamp_ns", 0))
    if gps > 0:
        return gps, "first_gps_timestamp_ns (insv RTC)"
    return int(d["record_start_host_ns"]), "record_start_host_ns (biased)"


def load_offset_sec(offset_file: Path | None) -> float:
    if not offset_file or not offset_file.exists():
        return 0.0
    for line in offset_file.read_text().splitlines():
        if line.startswith("offset_sec="):
            return float(line.split("=", 1)[1])
    return 0.0


def probe_mp4_pts(mp4: Path) -> list[float]:
    """
    ffprobe → 每帧 pts_time (秒, 相对视频起点)
    用 packet=pts_time（frame=pts_time 在某些 ffprobe 版本上会返回空）
    packet 顺序不是显示顺序，最后 sort 一次。
    """
    cmd = [
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "packet=pts_time", "-of", "csv=p=0", str(mp4),
    ]
    out = subprocess.check_output(cmd).decode()
    pts = []
    for line in out.splitlines():
        line = line.strip()
        if not line or line == "N/A": continue
        try:
            pts.append(float(line))
        except ValueError:
            continue
    return sorted(pts)


def extract_frames_to_dir(mp4: Path, tmp_dir: Path, jpeg_quality: int = 2) -> None:
    """ffmpeg 抽全部帧到 tmp_dir/<8-digit>.jpg，PTS 顺序保持。"""
    subprocess.check_call([
        "ffmpeg", "-loglevel", "error", "-y", "-i", str(mp4),
        "-vsync", "0", "-qscale:v", str(jpeg_quality),
        str(tmp_dir / "%08d.jpg"),
    ])


def subsample_pts_by_fps(pts: list[float], target_fps: float | None) -> list[int]:
    """按 target_fps 时间等间隔挑选帧 index (0-based)。None = 全要。"""
    if not target_fps or target_fps <= 0:
        return list(range(len(pts)))
    keep = []
    last_t = -1e18
    min_dt = 1.0 / target_fps
    for i, t in enumerate(pts):
        if t - last_t >= min_dt:
            keep.append(i)
            last_t = t
    return keep


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hera_db3", type=Path, required=True,
                    help="hera-desktop 转好的 rosbag2 db3 文件 或它所在目录 (含 metadata.yaml)")
    ap.add_argument("--panorama", type=Path, required=True,
                    help="预拼接的 panorama.mp4 (ERP)")
    ap.add_argument("--session", type=Path, required=True,
                    help="session.json (必须已跑 patch_session_with_gyro_bridge.py)")
    ap.add_argument("--offset", type=Path, default=None,
                    help="sync_offset.txt (可选, 无则 offset_sec=0)")
    ap.add_argument("--output_bag", type=Path, required=True,
                    help="输出 ROS2 bag 目录 (会自动创建, 已存在会报错保护)")
    ap.add_argument("--target_fps", type=float, default=1.0,
                    help="ERP 帧下采样率 (默认 1fps, 标定不需要多)")
    ap.add_argument("--image_topic", type=str, default="/insta/image")
    ap.add_argument("--camera_info_topic", type=str, default="/insta/camera_info")
    args = ap.parse_args()

    # 1) 依赖检查
    try:
        from rosbags.rosbag2 import Reader, Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        sys.exit("[error] 请先: pip install rosbags")
    try:
        import cv2
        import numpy as np
    except ImportError:
        sys.exit("[error] 请先: pip install opencv-python numpy")

    if args.output_bag.exists():
        sys.exit(f"[error] 输出目录已存在, 请删除或换名: {args.output_bag}")

    # 2) 解析 hera_db3 路径（用户可能给 .db3 或它所在目录）
    if args.hera_db3.is_file() and args.hera_db3.suffix == ".db3":
        src_bag_dir = args.hera_db3.parent
    else:
        src_bag_dir = args.hera_db3
    if not (src_bag_dir / "metadata.yaml").exists():
        sys.exit(f"[error] {src_bag_dir}/metadata.yaml 不存在, 不是有效 ROS2 bag")

    # 3) 读锚点 & offset
    anchor_ns, anchor_src = load_video_anchor_ns(args.session)
    offset_sec = load_offset_sec(args.offset)
    print(f"[t0 ] video anchor = {anchor_ns}  ({anchor_src})")
    print(f"[t0 ] sync offset  = {offset_sec:+.6f} s")
    print(f"[t0 ] 视频每帧 host_ns = anchor + PTS·1e9 + offset·1e9")

    # 4) 抽帧
    print(f"[img] ffprobe {args.panorama} ...")
    pts = probe_mp4_pts(args.panorama)
    print(f"[img] {len(pts)} frames, video 时长 {pts[-1]-pts[0]:.2f}s")

    keep_idx = subsample_pts_by_fps(pts, args.target_fps)
    print(f"[img] 下采样到 {args.target_fps} fps → 保留 {len(keep_idx)} 帧")

    tmp = Path(tempfile.mkdtemp(prefix="hera2bag_"))
    print(f"[img] ffmpeg 抽帧到 {tmp} ...")
    extract_frames_to_dir(args.panorama, tmp)

    # 5) 读第一帧探图像 size
    first_jpg = tmp / f"{keep_idx[0]+1:08d}.jpg"   # ffmpeg 从 1 起
    img0 = cv2.imread(str(first_jpg))
    if img0 is None:
        sys.exit(f"[error] 无法读 {first_jpg}")
    H, W = img0.shape[:2]
    print(f"[img] frame size = {W} × {H}")

    # 6) 打开源 bag + 建目标 bag
    print(f"\n[bag] 源 lidar bag: {src_bag_dir}")
    print(f"[bag] 输出 bag:     {args.output_bag}")
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    Image = typestore.types["sensor_msgs/msg/Image"]
    CameraInfo = typestore.types["sensor_msgs/msg/CameraInfo"]
    Header = typestore.types["std_msgs/msg/Header"]
    Time = typestore.types["builtin_interfaces/msg/Time"]
    RegionOfInterest = typestore.types["sensor_msgs/msg/RegionOfInterest"]

    reader = Reader(src_bag_dir)
    reader.open()
    writer = Writer(args.output_bag)
    # 需要在 open() 前设 compression（rosbags 库限制）
    try:
        writer.set_compression(writer.CompressionMode.NONE, "")
    except AttributeError:
        pass   # 老版 rosbags 无此 API, 默认就是无压缩
    writer.open()
    try:
        # 6a) 把源 bag 里所有 connection 原样开到目标 bag
        conn_map = {}
        for conn in reader.connections:
            new_conn = writer.add_connection(
                topic=conn.topic,
                msgtype=conn.msgtype,
                typestore=typestore,
            )
            conn_map[conn.id] = new_conn
            print(f"[bag] 复用 topic: {conn.topic} ({conn.msgtype})")

        # 6b) 加 image + camera_info 两个新 connection
        img_conn = writer.add_connection(
            topic=args.image_topic,
            msgtype="sensor_msgs/msg/Image",
            typestore=typestore,
        )
        info_conn = writer.add_connection(
            topic=args.camera_info_topic,
            msgtype="sensor_msgs/msg/CameraInfo",
            typestore=typestore,
        )
        print(f"[bag] 新增 topic: {args.image_topic} (sensor_msgs/msg/Image)")
        print(f"[bag] 新增 topic: {args.camera_info_topic} (sensor_msgs/msg/CameraInfo)")

        # 6c) 复制源 bag 里所有 lidar 消息
        n_copied = 0
        for conn, ts, raw in reader.messages():
            writer.write(conn_map[conn.id], ts, raw)
            n_copied += 1
        print(f"\n[bag] 已复制 {n_copied} 条 lidar 消息 (原时戳保留)")

        # 6d) 写入 image + camera_info（按 host_ns 时序）
        n_img = 0
        anchor_sec = anchor_ns / 1e9
        for i in keep_idx:
            src_jpg = tmp / f"{i+1:08d}.jpg"
            img = cv2.imread(str(src_jpg))
            if img is None:
                print(f"[warn] 缺帧 {src_jpg}, 跳过"); continue

            host_sec = anchor_sec + pts[i] + offset_sec
            host_ns = int(round(host_sec * 1e9))

            sec = host_ns // 1_000_000_000
            nanosec = host_ns % 1_000_000_000
            stamp = Time(sec=sec, nanosec=nanosec)
            header = Header(stamp=stamp, frame_id="insta_erp")

            # BGR → sensor_msgs/msg/Image (bgr8)
            # rosbags 序列化 uint8[] 要 numpy array (有 .view() 方法)
            img_msg = Image(
                header=header,
                height=H, width=W,
                encoding="bgr8",
                is_bigendian=0,
                step=W * 3,
                data=np.ascontiguousarray(img).view(np.uint8).reshape(-1),
            )

            # CameraInfo (equirectangular 语义标记; direct_visual_lidar_calibration
            # 里如果传了 --camera_model equirectangular, 会忽略 intrinsics 用图像 size)
            info_msg = CameraInfo(
                header=header,
                height=H, width=W,
                distortion_model="equirectangular",
                d=np.array([], dtype=np.float64),
                k=np.array([float(W), 0.0, W/2.0,
                            0.0, float(H), H/2.0,
                            0.0, 0.0, 1.0], dtype=np.float64),
                r=np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64),
                p=np.array([float(W), 0.0, W/2.0, 0.0,
                            0.0, float(H), H/2.0, 0.0,
                            0.0, 0.0, 1.0, 0.0], dtype=np.float64),
                binning_x=0, binning_y=0,
                roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
            )

            img_raw = typestore.serialize_cdr(img_msg, "sensor_msgs/msg/Image")
            info_raw = typestore.serialize_cdr(info_msg, "sensor_msgs/msg/CameraInfo")
            writer.write(img_conn, host_ns, img_raw)
            writer.write(info_conn, host_ns, info_raw)
            n_img += 1

        print(f"[bag] 已写入 {n_img} 帧 image + camera_info (topic: {args.image_topic})")
    finally:
        writer.close()
        reader.close()

    # 7) 清理临时目录
    import shutil
    shutil.rmtree(tmp, ignore_errors=True)

    print(f"\n[out] 输出 bag → {args.output_bag}")
    print(f"[out] metadata.yaml 检查:")
    print(f"      cat {args.output_bag}/metadata.yaml")
    print()
    print(f"下一步（Docker 内）:")
    print(f"  ros2 run direct_visual_lidar_calibration preprocess_ros2 \\")
    print(f"    <this_output_dir_parent> <output_preprocessed> \\")
    print(f"    --camera_model equirectangular \\")
    print(f"    --image_topic {args.image_topic} \\")
    print(f"    --points_topic /lidar/mid360/point_cloud2 \\")
    print(f"    --camera_info_topic {args.camera_info_topic}")


if __name__ == "__main__":
    main()
