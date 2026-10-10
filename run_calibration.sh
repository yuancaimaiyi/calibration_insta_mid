#!/bin/bash
# -----------------------------------------------------------------------------
# Vistara 头盔 · Insta360 + Mid360 外参标定一条龙
#
# 期望目录结构 ($DATA_ROOT 下):
#   1/
#     <stem>.hera
#     <stem>.insv
#     <stem>.session.json      <- hera-desktop 原始产出
#     out.db3                  <- hera-desktop 转的 rosbag
#     metadata.yaml
#     panorama.mp4             <- hera-desktop 用 template 算法拼
#   2/ ...
#   N/ ...
#
# 输出:
#   $DATA_ROOT/bags/scan_XX_bag/           Step C 产出
#   $DATA_ROOT/calibration_result/         Step D 产出 (preprocess + match + calib)
#   $DATA_ROOT/T_lidar_camera.txt          Step E 出厂 T_lc (7 数一行)
#
# 验收:
#   脚本结束前自动打开 viewer (需 X / xhost +local:root)
#   或手动看 $DATA_ROOT/calibration_result/scan_*_bag_superglue.png
# -----------------------------------------------------------------------------
set -euo pipefail

usage() {
  cat <<EOF
用法:
  $0 <DATA_ROOT> [--no-viewer] [--helmet-dir PATH] [--docker-image IMAGE]

参数:
  DATA_ROOT       根目录, 内含 1/, 2/, ... N/ 每站子目录
  --no-viewer     跳过 Step F 可视化验收
  --helmet-dir    helmet 脚本目录 (默认: 本脚本所在目录)
  --docker-image  dvlc 镜像 (默认 dvlc:rgb; 想要灰度验收用 dvlc:superglue)

示例:
  $0 /home/me/calibration_1015
  $0 /home/me/calibration_1015 --no-viewer --docker-image dvlc:superglue
EOF
  exit 1
}

[[ $# -lt 1 ]] && usage
DATA_ROOT=$(readlink -f "$1"); shift
RUN_VIEWER=1
HELMET_DIR=$(dirname "$(readlink -f "$0")")
DOCKER_IMAGE=dvlc:rgb

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-viewer)       RUN_VIEWER=0; shift ;;
    --helmet-dir)      HELMET_DIR=$(readlink -f "$2"); shift 2 ;;
    --docker-image)    DOCKER_IMAGE="$2"; shift 2 ;;
    -h|--help)         usage ;;
    *)                 echo "未知参数: $1"; usage ;;
  esac
done

[[ -d "$DATA_ROOT" ]] || { echo "DATA_ROOT 不存在: $DATA_ROOT"; exit 1; }
[[ -f "$HELMET_DIR/patch_session_with_gyro_bridge.py" ]] || { echo "找不到 helmet 脚本: $HELMET_DIR"; exit 1; }

# 枚举子目录 (纯数字命名, 字典序)
STATIONS=($(cd "$DATA_ROOT" && ls -d [0-9]*/ 2>/dev/null | sed 's|/||' | sort -n))
[[ ${#STATIONS[@]} -ge 3 ]] || { echo "至少需要 3 站, 当前 ${#STATIONS[@]} 站"; exit 1; }
N=${#STATIONS[@]}

echo "=============================================================="
echo " Vistara 头盔 · 外参标定"
echo "   DATA_ROOT     $DATA_ROOT"
echo "   HELMET_DIR    $HELMET_DIR"
echo "   DOCKER_IMAGE  $DOCKER_IMAGE"
echo "   N stations    $N  (${STATIONS[*]})"
echo "=============================================================="

# -----------------------------------------------------------------------------
# Step B1 · 每站 gyro bridge patch (给 session.json 打 frame_0_host_ns)
# -----------------------------------------------------------------------------
echo
echo "### Step B1 · gyro bridge patch × $N ###"
for s in "${STATIONS[@]}"; do
  d="$DATA_ROOT/$s"
  HERA=$(ls "$d"/*.hera 2>/dev/null | head -n1 || true)
  INSV=$(ls "$d"/*.insv 2>/dev/null | head -n1 || true)
  SESSION=$(ls "$d"/*.session.json 2>/dev/null | head -n1 || true)
  [[ -z "$HERA" || -z "$INSV" || -z "$SESSION" ]] && { echo "[$s] 缺 hera/insv/session.json"; exit 1; }
  echo "--- 站 $s ---"
  python3 "$HELMET_DIR/patch_session_with_gyro_bridge.py" \
    --hera    "$HERA" \
    --insv    "$INSV" \
    --session "$SESSION" | tail -8
done

# -----------------------------------------------------------------------------
# Step C · hera_to_rosbag.py (打包 ROS2 bag, 含 /insta/image + /insta/camera_info)
# -----------------------------------------------------------------------------
echo
echo "### Step C · hera_to_rosbag × $N ###"
BAGS_DIR="$DATA_ROOT/bags"
rm -rf "$BAGS_DIR"; mkdir -p "$BAGS_DIR"
for s in "${STATIONS[@]}"; do
  d="$DATA_ROOT/$s"
  SESSION=$(ls "$d"/*.session.json | head -n1)
  DB3="$d/out.db3"
  MP4="$d/panorama.mp4"
  [[ -f "$DB3" ]] || { echo "[$s] 缺 out.db3"; exit 1; }
  [[ -f "$MP4" ]] || { echo "[$s] 缺 panorama.mp4"; exit 1; }
  BAG_OUT="$BAGS_DIR/scan_$(printf %02d "$s")_bag"
  echo "--- 站 $s → $(basename "$BAG_OUT") ---"
  python3 "$HELMET_DIR/hera_to_rosbag.py" \
    --hera_db3   "$DB3" \
    --panorama   "$MP4" \
    --session    "$SESSION" \
    --output_bag "$BAG_OUT" \
    --target_fps 0.5 | grep -E '^\[out\]|^\[err\]' | tail -3
done

# -----------------------------------------------------------------------------
# Step D · docker 跑 direct_visual_lidar_calibration (preprocess + match + calibrate)
# -----------------------------------------------------------------------------
echo
echo "### Step D · dvlc docker pipeline ###"
# 旧产物是 docker 以 root 建的, 普通用户删不动; 用 docker 清理
docker run --rm -v "$DATA_ROOT:/data" "$DOCKER_IMAGE" bash -c "rm -rf /data/calibration_result" || true

docker run --rm --gpus all -v "$DATA_ROOT:/data" "$DOCKER_IMAGE" bash -c "
  set -e
  apt-get update -qq && apt-get install -y -qq xvfb 2>&1 | tail -1

  # D1 preprocess
  echo '--- D1 preprocess ---'
  ros2 run direct_visual_lidar_calibration preprocess \\
      --data_path /data/bags --dst_path /data/calibration_result \\
      --auto_topic --camera_model equirectangular \\
      --image_topic /insta/image --points_topic /lidar/mid360/point_cloud2 \\
      --camera_info_topic /insta/camera_info --dynamic_lidar_integration

  # D2 SuperGlue 匹配 (indoor 室内, 户外改 outdoor; rotate_camera 固定 0 别扫, 旋转不变性差)
  echo '--- D2 find_matches_superglue ---'
  ros2 run direct_visual_lidar_calibration find_matches_superglue.py \\
      /data/calibration_result --rotate_camera 0 --superglue indoor

  # D3 RANSAC + LSQ 粗估 initial guess
  echo '--- D3 initial_guess_auto ---'
  ros2 run direct_visual_lidar_calibration initial_guess_auto /data/calibration_result

  # D4 NID 精化 (xvfb 绕 GLFW)
  echo '--- D4 calibrate (NID) ---'
  xvfb-run -a --server-args='-screen 0 1024x768x24' \\
    ros2 run direct_visual_lidar_calibration calibrate /data/calibration_result --background --auto_quit

  echo '--- final calib.json ---'
  cat /data/calibration_result/calib.json
"

# -----------------------------------------------------------------------------
# Step E · 出厂 T_lidar_camera.txt (7 数一行)
# -----------------------------------------------------------------------------
echo
echo "### Step E · 出厂 T_lidar_camera.txt ###"
python3 -c "
import json
d = json.load(open('$DATA_ROOT/calibration_result/calib.json'))
r = d['results']['T_lidar_camera']
print(' '.join(f'{v:.9f}' for v in r))
" > "$DATA_ROOT/T_lidar_camera.txt"
echo "T_lc:  $(cat "$DATA_ROOT/T_lidar_camera.txt")"

# 简单量级 sanity (|t| 应该 ~几 cm 到十几 cm)
python3 - <<PY
import math, json
d = json.load(open('$DATA_ROOT/calibration_result/calib.json'))
tx,ty,tz,qx,qy,qz,qw = d['results']['T_lidar_camera']
t = math.sqrt(tx*tx + ty*ty + tz*tz)
print(f'|t| = {t*100:.2f} cm')
if t > 0.5:
    print('⚠️  |t| > 50 cm, 结果大概率跑飞, 不建议出厂!')
    print('   典型根因: SP+SG 有效匹配不足, 需按 calibration.md Step A v2 重采.')
elif t < 0.03:
    print('⚠️  |t| < 3 cm, 偏小, 跟物理量测核对一下.')
else:
    print('✅ |t| 在合理区间 (3-50 cm).')
PY

# -----------------------------------------------------------------------------
# Step F · 可视化验收
# -----------------------------------------------------------------------------
echo
echo "### Step F · 可视化 ###"
echo "SuperGlue 匹配图 (每站一张):"
ls "$DATA_ROOT/calibration_result"/scan_*_bag_superglue.png | sed 's|^|  |'

if [[ "$RUN_VIEWER" -eq 1 ]]; then
  echo
  echo "启动 dvlc viewer (需本机 X 显示; 远端请加 --no-viewer)..."
  xhost +local:root >/dev/null 2>&1 || true
  docker run --rm --gpus all \
    --env DISPLAY="${DISPLAY:-:0}" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --network=host \
    -v "$DATA_ROOT:/data" "$DOCKER_IMAGE" \
    ros2 run direct_visual_lidar_calibration viewer /data/calibration_result || \
    echo "viewer 启动失败, 远端请 ssh -X 或直接看 superglue.png"
fi

echo
echo "=============================================================="
echo " ✅ 全流程完成"
echo "   T_lc 文件:   $DATA_ROOT/T_lidar_camera.txt"
echo "   验收: 检查 viewer 点云投影, 或看 scan_*_bag_superglue.png"
echo "=============================================================="
