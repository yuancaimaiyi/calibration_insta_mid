# Vistara 头盔 · Insta360 + Mid360 外参标定完整流程

**首次标定 T_lidar_camera 权威文档**（一次标定, 终生复用；出厂随机器交付 `T_lidar_camera.txt`）。
2026-09-29 定版, 基于 5 站实测走通。覆盖并作废 `VISTARA_VPS_INPUT.md` Step A-F 里 preprocess/calibrate 那两段命令。

---

## TL;DR

```
5-10 站静止扫描
   │
   ├─ B1  patch_session_with_gyro_bridge.py × N     (每站, 打 frame_0_host_ns)
   ├─ B2  hera-desktop / hera-convert   .hera → out.db3
   └─ B3  hera-desktop / stitcher       .insv → panorama.mp4
       │
       └─ C  hera_to_rosbag.py × N       (每站, 打包成 ROS2 bag; --target_fps 0.5)
           │
           └─ D  docker run dvlc:rgb bash -c 'preprocess+find_matches+init+calibrate'
               │
               ├─ E  python3 提 calib.json → T_lidar_camera.txt (7 数)
               └─ F  docker run dvlc:rgb viewer   (X GUI, 真 RGB 上色验收)
```

**~30-60 min 全程**（含 SuperGlue CPU 匹配 + NID 精化）。

---

## 一次性: 制作两个 Docker 镜像

### 前置

- Docker + nvidia-container-toolkit（GPU 可选, CPU torch 1.8 够用）
- 官方镜像：`docker pull docker.1ms.run/koide3/direct_visual_lidar_calibration:humble`（3.94 GB）

### Dockerfile.superglue (base → 加 torch + SuperGlue)

```dockerfile
FROM docker.1ms.run/koide3/direct_visual_lidar_calibration:humble

# 阿里云 apt 镜像
RUN sed -i 's|http://archive.ubuntu.com|http://mirrors.aliyun.com|g; s|http://security.ubuntu.com|http://mirrors.aliyun.com|g' /etc/apt/sources.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      git python3-pip python3-numpy python3-torch python3-torchvision \
      python3-matplotlib python3-opencv \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# gitclone.com 镜像加速 SuperGlue clone (fallback: gitmirror.com)
RUN (git clone --depth=1 https://gitclone.com/github.com/magicleap/SuperGluePretrainedNetwork /root/SuperGlue \
      || git clone --depth=1 https://hub.gitmirror.com/https://github.com/magicleap/SuperGluePretrainedNetwork /root/SuperGlue)

ENV PYTHONPATH=/root/SuperGlue:${PYTHONPATH}
```

```bash
docker build -f Dockerfile.superglue -t dvlc:superglue .   # ~5 min
```

### Dockerfile.rgb (dvlc:superglue → 加 RGB 上色 patch)

```dockerfile
FROM dvlc:superglue

# 1) preprocess_ros2: 保留 BGR
RUN sed -i 's/"mono8"/"bgr8"/g' \
    /root/ros2_ws/src/direct_visual_lidar_calibration/src/preprocess_ros2.cpp

# 2) visual_lidar_data: 用 IMREAD_COLOR 读回 3ch
RUN sed -i 's|cv::imread(data_path + "/" + bag_name + ".png", 0);|cv::imread(data_path + "/" + bag_name + ".png", cv::IMREAD_COLOR);|' \
    /root/ros2_ws/src/direct_visual_lidar_calibration/src/vlcal/common/visual_lidar_data.cpp

# 3) visualizer: 3ch 直用, 1ch 才转
RUN sed -i 's|cv::cvtColor(dataset\[selected_bag_id\]->image, bgr, cv::COLOR_GRAY2BGR);|bgr = dataset[selected_bag_id]->image.channels() == 3 ? dataset[selected_bag_id]->image : (cv::cvtColor(dataset[selected_bag_id]->image, bgr, cv::COLOR_GRAY2BGR), bgr);|' \
    /root/ros2_ws/src/direct_visual_lidar_calibration/src/vlcal/common/visual_lidar_visualizer.cpp

# 4) points_color_updater: 采 Vec3b BGR → Eigen RGB (兼容 1ch)
RUN sed -i \
    -e 's|const unsigned char pix = image.at<std::uint8_t>(pt_2d.y(), pt_2d.x());|const auto pix = image.channels() == 3 ? image.at<cv::Vec3b>(pt_2d.y(), pt_2d.x()) : cv::Vec3b(image.at<std::uint8_t>(pt_2d.y(), pt_2d.x()), image.at<std::uint8_t>(pt_2d.y(), pt_2d.x()), image.at<std::uint8_t>(pt_2d.y(), pt_2d.x()));|' \
    -e 's|const Eigen::Vector4f color(pix / 255.0f, pix / 255.0f, pix / 255.0f, 1.0f);|const Eigen::Vector4f color(pix[2] / 255.0f, pix[1] / 255.0f, pix[0] / 255.0f, 1.0f);|' \
    /root/ros2_ws/src/direct_visual_lidar_calibration/src/vlcal/common/points_color_updater.cpp

# 4b) preprocess.cpp equalizeHist 只支持 1ch —— BGR → YCrCb → 均衡 Y → BGR
RUN sed -i 's|cv::equalizeHist(image.clone(), image);|if (image.channels() == 3) { cv::Mat ycrcb; cv::cvtColor(image, ycrcb, cv::COLOR_BGR2YCrCb); std::vector<cv::Mat> ch; cv::split(ycrcb, ch); cv::equalizeHist(ch[0].clone(), ch[0]); cv::merge(ch, ycrcb); cv::cvtColor(ycrcb, image, cv::COLOR_YCrCb2BGR); } else { cv::equalizeHist(image.clone(), image); }|' \
    /root/ros2_ws/src/direct_visual_lidar_calibration/src/vlcal/preprocess/preprocess.cpp

# 4c) NID cost 3ch bug ⚠️ 关键 —— image.at<uint8_t> 在 BGR 3ch 只读 Blue 通道,
#     NID cost 完全跑偏, T_lc 崩 (实测 |t| 从 4.6cm → 41cm, 方向乱)
#     必须转 Rec.601 luminance
RUN sed -i 's|const double pixel = image.at<std::uint8_t>(pt_2d.y(), pt_2d.x()) / 255.0;|double pixel; if (image.channels() == 3) { const cv::Vec3b\& p = image.at<cv::Vec3b>(pt_2d.y(), pt_2d.x()); pixel = (0.114 * p[0] + 0.587 * p[1] + 0.299 * p[2]) / 255.0; } else { pixel = image.at<std::uint8_t>(pt_2d.y(), pt_2d.x()) / 255.0; }|' \
    /root/ros2_ws/src/direct_visual_lidar_calibration/src/vlcal/calib/cost_calculator_nid.cpp

# 4d) NIDCost normalized_image ⚠️ 关键 —— convertTo 不改通道数, 3ch 情况下
#     后续 at<double> 也只取第 1 通道 (Blue). 必须显式 BGR→Gray
RUN sed -i 's|dataset\[i\]->image.convertTo(normalized_image, CV_64FC1, 1.0 / 255.0);|cv::Mat gray_source = dataset[i]->image; if (gray_source.channels() == 3) cv::cvtColor(gray_source, gray_source, cv::COLOR_BGR2GRAY); gray_source.convertTo(normalized_image, CV_64FC1, 1.0 / 255.0);|' \
    /root/ros2_ws/src/direct_visual_lidar_calibration/src/vlcal/calib/visual_camera_calibration.cpp

WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select direct_visual_lidar_calibration --cmake-args -DCMAKE_BUILD_TYPE=Release"
```

```bash
docker build -f Dockerfile.rgb -t dvlc:rgb .   # ~1 min (只有 sed + colcon)
```

**两个 Dockerfile 归档在** `/home/yuancaimaiyi/feima_data/calibration/Dockerfile.{superglue,rgb}`。

---

## Step A · 采集 5-10 个静止点位

**操作**（跟日常 VPS 采集流程一样, 只是要求静止 + 多次换视角）：

1. 手机 web 端点录制 button
2. 头盔戴稳, **保持静止 5-8 秒**（**上限 8s! 太长 CT-ICP 累积微动 drift**, 单站点云会 smear, 视觉验收看不清）
3. 点停止, 等 15-20 秒下载 `.insv`
4. 挪 2-5 m 换个点位、换个朝向, 重复
5. 总共 **5-10 站**

**点位选择**（决定 SuperGlue 匹配质量）：
- **场景纹理丰富**: 家具/柱子/棋盘打印贴墙/管道/门框
- **Mid360 intensity 强的目标**: 反光条 / 白纸 / 金属栅栏 / AprilTag
- **每站视角差异越大越好**: 换朝向不换位置也算, 别 5 站站同一方向
- **室内 4x4 m+ 空间**, 目标距离 1.5-3 m（近场 Mid360 稀疏, 远场纹理少）

**产出**（每站一份, N=5-10）：
```
<stem_i>.hera + <stem_i>.insv + <stem_i>.session.json
```

---

## Step B · 每份数据预处理（B1 + B2 + B3）

### B1 · gyro bridge patch（每站一次）

```bash
for i in $(seq 1 N); do
  STEM=$(ls /path/to/calibration/$i/*.hera | xargs -n1 basename -s .hera)
  python3 /path/to/helmet/patch_session_with_gyro_bridge.py \
    --hera    /path/to/calibration/$i/${STEM}.hera \
    --insv    /path/to/calibration/$i/${STEM}.insv \
    --session /path/to/calibration/$i/${STEM}.session.json
done
```

**必检**（每份）：
- `offset std < 10 ms`
- `Δ = frame_0 - record_start = 正值 (+几百 ms 到 +1500 ms)` — 负值 = 单位错
- 打印里出现 `[out] patched → …session.json` = 成功

### B2 · .hera → ROS2 db3

用 hera-desktop GUI 打开每份 `.hera` 走"转 ROS2 bag"流程, 产出 `<stem_i>/out.db3 + metadata.yaml`, 含 topic:
- `/lidar/mid360/point_cloud2` (sensor_msgs/msg/PointCloud2)
- `/lidar/mid360/imu` (sensor_msgs/msg/Imu)

（`hera-convert` CLI 在部分环境 libboost 版本冲突 broken, 无法批处理; 只能 GUI 手动过一次。）

### B3 · .insv → panorama.mp4

用 hera-desktop 或 Insta360 SDK stitcher 把 `.insv` 拼成 ERP 全景视频（`3840×1920` 或原生分辨率）, 产出 `<stem_i>/panorama.mp4`。

---

## Step C · hera_to_rosbag.py × N（打包标定 bag）

```bash
mkdir -p /path/to/calibration/bags
for i in $(seq 1 N); do
  STEM=$(ls /path/to/calibration/$i/*.hera | xargs -n1 basename -s .hera)
  python3 /path/to/helmet/hera_to_rosbag.py \
    --hera_db3   /path/to/calibration/$i/out.db3 \
    --panorama   /path/to/calibration/$i/panorama.mp4 \
    --session    /path/to/calibration/$i/${STEM}.session.json \
    --output_bag /path/to/calibration/bags/scan_$(printf %02d $i)_bag \
    --target_fps 0.5    # 静止扫描 2s 一帧够
done
```

**关键参数**：
- `--target_fps 0.5` —— 静止场景冗余帧无信息, 减少 SuperGlue 计算量 (~10x)
- **不需要 `--offset`** —— Mid360 静止, sync 无从算; 缺省 offset=0 即可 (Insta host_ns 由 gyro bridge 打好, Mid360 hera 原生 host_ns)

**验证**：`bags/scan_*_bag/metadata.yaml` 每份含 4 个 topic:
- `/lidar/mid360/point_cloud2` (73-135 帧)
- `/lidar/mid360/imu`
- `/insta/image` (3-9 帧 ERP 3840×1920)
- `/insta/camera_info`

---

## Step D · Docker 跑 direct_visual_lidar_calibration

**GPU 机器**（如果无 GPU 去掉 `--gpus all`, CPU 也行 33 张图 ~10 min）：

```bash
docker run --rm --gpus all \
    -v /path/to/calibration:/data \
    dvlc:rgb bash -c "
      set -e
      apt-get update -qq && apt-get install -y -qq xvfb 2>&1 | tail -1

      # D1. preprocess (注意 --dynamic_lidar_integration 必需, Mid360 non-repetitive)
      ros2 run direct_visual_lidar_calibration preprocess \
          --data_path /data/bags --dst_path /data/out \
          --auto_topic --camera_model equirectangular \
          --image_topic /insta/image --points_topic /lidar/mid360/point_cloud2 \
          --camera_info_topic /insta/camera_info --dynamic_lidar_integration

      # D2. SuperGlue 匹配 (indoor weights 对室内办公场景; 户外用 outdoor)
      ros2 run direct_visual_lidar_calibration find_matches_superglue.py \
          /data/out --rotate_camera 0 --superglue indoor

      # D3. RANSAC + LSQ 出 initial guess
      ros2 run direct_visual_lidar_calibration initial_guess_auto /data/out

      # D4. NID 精化 (xvfb 绕 GLFW init bug)
      xvfb-run -a --server-args='-screen 0 1024x768x24' \
        ros2 run direct_visual_lidar_calibration calibrate /data/out --background --auto_quit

      cat /data/out/calib.json
    "
```

**必检输出**：
- preprocess: `LiDAR FoV: ~179°`, 5 站都 `processed`
- find_matches: 5 站都 `Loaded SuperGlue model`, 无 crash
- initial_guess: `num_inliers > 20`（<10 就是场景差, 参见 Step G 排查）
- calibrate: Ceres 收敛 (`Termination: CONVERGENCE`, 最后 Δ < 1e-4)
- `calib.json` 里出现 `T_lidar_camera` 字段

**⚠️ 五大坑记录**（每一个我都踩过）：
1. 工具名 `preprocess`, **不是** `preprocess_ros2`
2. 官方 docker 缺 torch/SuperGlue, **必须**用 `dvlc:superglue` 或 `dvlc:rgb`
3. `calibrate` 即使加 `--background` 也硬要 X, **必须** `xvfb-run` 包一层
4. `--dynamic_lidar_integration` **必须开**, Mid360 non-repetitive 单帧稀疏, 去掉直接崩 (T_lc 差 60+ cm)
5. `preprocess` 保存的 `scan_XX_bag.png` 是灰度 —— viewer 上色也只用灰度; 用 `dvlc:rgb` (改了 4 处 C++) 才能 RGB 上色

---

## Step E · 出厂 T_lidar_camera.txt

```bash
python3 -c "
import json
d = json.load(open('/path/to/calibration/out/calib.json'))
print(' '.join(f'{v:.9f}' for v in d['results']['T_lidar_camera']))
" > /path/to/calibration/T_lidar_camera.txt
```

**格式**（一行, 7 数, 空格分隔）：
```
tx ty tz qx qy qz qw
-0.034334212 -0.007812447 0.029323380 -0.396648179 0.402956761 -0.586663504 0.579760298
```

这个文件**随头盔出厂, 后续所有 `panorama_to_prior_poses.py --extrinsic_file` 直接读**。

---

## Step F · 视觉验收（关键, 决定是否出厂）

### 有 GUI（本机或 ssh -X）

```bash
xhost +local:root      # 一次

docker run --rm --gpus all --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --network=host \
    -v /home/yuancaimaiyi/feima_data/calibration:/data \
    dvlc:rgb \
    ros2 run direct_visual_lidar_calibration viewer /data/calibration_result_v2
```

**看什么**：

1. **左侧 3D 点云窗口**（RGB 上色, 用 `dvlc:rgb`）：
   - 拖动转视角, 看点云的颜色 vs 场景真实颜色是否一致（墙是白/绿的、椅子红/黑的、纸箱黄色的）
   - 颜色错位 = T_lc 错

2. **右上角 image tab / Image 0-4**（可切换 bag）：
   - 逐站看 LiDAR 点投到 ERP 图上的重投影
   - **近景 1-2 m** (桌腿/椅背/棋盘): 应贴 <5 px ✅
   - **中距 3-5 m** (门框/柱子): 应对齐 ✅
   - **远景 >10 m**: 允许几像素偏（LiDAR 稀疏正常）忽略
   - **全部朝一个方向偏 → 旋转错**
   - **近处贴远处飘 → 平移错**

### 无 GUI（远端服务器）

导出 5 张 overlay PNG:
```bash
ls /path/to/calibration/out/scan_*_bag_superglue.png   # SuperGlue 匹配可视化
# 或用自己写脚本投影 (参考 overlay.py)
```

---

## Step G · 判据 & Cross-validation

### 单次结果判据

| 指标 | 门槛 | 意义 |
|---|---|---|
| SuperGlue init inliers | > 20 (5-bag) | 场景纹理是否够 |
| NID 收敛 Δf | < 1e-4 | 优化是否稳 |
| NID final f (per-bag) | < 1.0 | 匹配 mutual info 强 |
| RANSAC inlier | > 30% | 场景可解性 |

### Cross-validation（推荐, 5 min 成本）

用 N-1 bag 重跑, 对比 T_lc:
```bash
mkdir bags_subset && cd bags_subset
ln -s ../bags/scan_02_bag . && ln -s ../bags/scan_03_bag . && ...   # 剔除 1 站
# 重跑 Step D 到 bags_subset → out_subset
```

**判据**：
- **Δt < 3 mm & Δrot < 0.1°** → T_lc 稳, 直接出厂
- **Δt > 1 cm 或 Δrot > 0.5°** → 剔除某站有大影响, 那站可能是 outlier, 查采集问题

**Vistara 2026-09-28 实测**: 5-bag vs 4-bag(剔 1 站) Δt=2.6 mm, Δrot=0.087° → 官方精度 0.5°/1-2 cm 通过 5x margin。

---

## Step H · 常见故障排查

| 症状 | 根因 | 处理 |
|---|---|---|
| `No executable found: preprocess_ros2` | 工具名错 | 用 `preprocess` |
| `ModuleNotFoundError: No module named 'torch'` | 用了官方镜像 | 换 `dvlc:superglue` 或 `dvlc:rgb` |
| `SuperGlue matches < 10` per bag | 场景纹理弱 / rotate 错 | 试 `--rotate_camera 90/180/270`, 或换点位重采 |
| `initial_guess_auto` RANSAC inlier < 5 | LiDAR intensity 太弱 | 找反光条/白墙场景, 增加视角多样性 |
| `calibrate` 报 `glfw error 65544: DISPLAY missing` | GLFW init bug | 用 `xvfb-run -a` 包一层 |
| `equalizeHist: _src.type() == CV_8UC1` | 用了 `dvlc:rgb` 但没打 YCrCb patch | 重建 dvlc:rgb, 确认 Dockerfile 有 equalizeHist patch |
| 5 bag NID 崩到 T_lc |t|>50cm 或方向反 | 忘了 `--dynamic_lidar_integration` | 加上 flag 重跑 |
| viewer 里点云全朝一个方向偏 | T_lc 旋转错 | Cross-val 或换点位重跑 |
| viewer 里点云近处贴远处飘 | T_lc 平移错 | Cross-val 或增加近距离目标 |
| 某一站视觉"漂浮/drift" | 那站采集 >10s, CT-ICP 累积微动 | 重采那站, 严格 5-8s |

---

## Step I · 采集时长 vs Drift（**要点, 别忽视**）

**理由**：`--dynamic_lidar_integration` 内部用 CT-ICP 累积 Mid360 non-repetitive 单帧 → 需要 sensor "尽量静止" 的假设; 时长越长, 人身微动累积成 pose drift, 点云被 smear。

| 时长 | 效果 |
|---|---|
| **5-8 秒** | ✅ 点云清晰, 视觉验收明确 |
| 10-15 秒 | ⚠️ 点云开始 smear, 单站 T_lc 不受影响 (联合优化能抵消), 但视觉验收看不清 |
| > 15 秒 | ❌ 明显 drift, 视觉验收失效 |

**Vistara 2026-09-28 站 1 采了 16.5s → 视觉"漂浮"; 站 2/3/4/5 都在 8-14s → 视觉都 OK; T_lc 出厂值不受影响（cross-val 通过）**。

---

## 附: 关键文件路径

| 文件 | 路径 |
|---|---|
| Dockerfile | `/home/yuancaimaiyi/feima_data/calibration/Dockerfile.{superglue,rgb}` |
| gyro bridge 脚本 | `helmet/patch_session_with_gyro_bridge.py` |
| bag 打包脚本 | `helmet/hera_to_rosbag.py` |
| Prior pose 生成 | `helmet/panorama_to_prior_poses.py --extrinsic_file <T_lc.txt>` |
| 兜底/debug 标定 | `helmet/calibrate_extrinsic.py` (已降级, 只做 sanity check 用, 不出厂) |

## 附: 决策历史 (为什么这套流程)

- **为什么用 direct_visual_lidar_calibration 而非 calibrate_extrinsic.py**: 后者依赖 SfM 相机 pose 质量, 平面运动场景 Z 不可观测, verdict 常年 SUSPECT (基线误差 5 cm); dvlc 直接 image-lidar 像素级 SuperGlue+NID, 精度 0.5°/1-2 cm。
- **为什么自建镜像 vs 官方 Dockerfile_with_superglue**: 官方从 gtsam 从头 build 30+ min; 我们从官方 build 好的 image 追加, ~5 min。
- **为什么改代码支持 RGB**: 灰度上色在验收环节难判"颜色错位", RGB 一眼看穿; NID 优化仍用灰度不影响精度。
