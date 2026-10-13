# calibration_insta_mid

**Vistara 头盔 · Insta360 X4 + Livox Mid360 外参标定 + 每次采集 prior pose 生成工具链**

本仓库解决两件事：
1. **首次标定**（一次性）：标定 Insta 相机 ↔ Mid360 LiDAR 的外参 `T_lidar_camera`，随设备绑定出厂
2. **每次采集**（常规）：根据这个外参 + GLIM 激光轨迹 + 全景视频，产出 `每帧 ERP 图 + prior pose`，直接喂下游 VPS 建图 / 稠密上色

---

## 仓库职责

```
硬件: Insta360 X4 + Livox Mid360 刚性共装头盔
    │
    ▼ 一次性 (出厂)
┌─────────────────────────────────────────────────┐
│ Step A-F   首次标定 T_lidar_camera               │
│                                                  │
│   5-10 站静止采集                                │
│   dvlc:rgb docker (本仓库 Dockerfile.*)          │
│   SuperGlue 匹配 + NID 精化                       │
│   calib.json → T_lidar_camera.txt (TUM, 7 数)    │
└─────────────────────────────────────────────────┘
    │ T_lidar_camera.txt 随设备交付
    ▼ 每次采集 (日常)
┌─────────────────────────────────────────────────┐
│ Step 1-2   每次采集 2 条命令                      │
│                                                  │
│   patch_session_with_gyro_bridge.py (锚第 0 帧) │
│   panorama_to_prior_poses.py        (抽帧+赋pose)│
│                                                  │
│   → frames/<host_ns>.jpg + prior_poses.txt       │
└─────────────────────────────────────────────────┘
    │
    ▼ 下游消费
┌─────────────────────────────────────────────────┐
│ VPS 建图 / vismap_mapper                         │
│ hera-desktop 「激光重建点云 RGB 着色」卡片       │
│ (后者需要本仓库产的 <stem>_pano/ 和 T_lc.txt)    │
└─────────────────────────────────────────────────┘
```

---

## 两条使用线

### A. 首次标定（出厂做一次）

**文档**：`calibration.md`（2026-09-29 定版，覆盖并替代旧版 `VISTARA_VPS_INPUT.md` 里 Step A-F 的 preprocess/calibrate 段）。

**流程**：
1. 做两个 Docker 镜像：`dvlc:superglue`（base + torch + SuperGlue）和 `dvlc:rgb`（+ RGB 支持）
2. 采集 5-10 站静止 .hera + .insv（纹理丰富场景）
3. 每站 `patch_session_with_gyro_bridge.py` 打锚 → `hera-convert` 转 .db3 → `panorama-stitch` 拼 .mp4 → `hera_to_rosbag.py` 打 ROS2 bag
4. `docker run dvlc:rgb bash -c 'preprocess+find_matches+init+calibrate'`
5. 从 `calib.json` 提 7 个数到 `T_lidar_camera.txt`
6. `dvlc:rgb viewer` 做 X GUI 真 RGB 上色验收（NID < 0.7 良好）

**耗时**：~30-60 min 全程（含 SuperGlue CPU 匹配 + NID 精化）。

### B. 每次采集（常规）

**文档**：`VISTARA_VPS_INPUT.md`。

```bash
# 1. 把 panorama.mp4 第 0 帧的 host 时间戳打进 session.json
python3 patch_session_with_gyro_bridge.py \
  --hera    X.hera --insv X.insv --session X.session.json

# 2. 抽帧 + 赋 prior pose
python3 panorama_to_prior_poses.py \
  --panorama       panorama.mp4 \
  --session        X.session.json \
  --traj           traj_lidar.txt \
  --offset         X_sync_offset.txt \
  --extrinsic_file T_lidar_camera.txt \
  --target_fps     30 \
  --out_dir        X_pano
```

**产物**：
```
X_pano/
├── frames/<host_ns>.jpg      # ERP 图像 (5760×2880 或 3840×1920)
├── prior_poses.txt           # image_name tx ty tz qx qy qz qw (T_world_camera)
└── frame_timestamps.csv
```

喂给下游 vismap / hera-desktop 的 RGB 着色卡片。

---

## 文件索引

| 文件 | 用途 |
|---|---|
| `README.md` | 本文件 — 仓库总览 |
| `calibration.md` | **首次标定权威文档**（Dockerfile + 流程 + 5 大踩坑点）|
| `VISTARA_VPS_INPUT.md` | **每次采集流程**（2 命令出 prior pose）|
| `patch_session_with_gyro_bridge.py` | gyro clock bridge 算视频第 0 帧 host_ns（精度 ±1-5 ms）|
| `panorama_to_prior_poses.py` | ffmpeg 抽帧 + 按 host_ns 命名 + 查 traj_lidar 插值 T_world_camera |
| `hera_to_rosbag.py` | 把 .hera (lidar+imu) + .mp4 (全景) 打包成 ROS2 bag (供 dvlc) |
| `calibration.sh` | 单站标定壳子脚本 |
| `run_calibration.sh` | 多站批量标定脚本 |
| `overlay.py` | RGB 投影验证（点云投到 ERP 看对齐）|
| `Dockerfile.superglue` | 构建 dvlc:superglue 镜像（dvlc + torch 1.8 + SuperGlue 权重）|
| `Dockerfile.rgb.deploy` | 构建 dvlc:rgb 镜像（上面 + RGB NID 修补）|

---

## 时间精度链

```
Insta 相机内钟 ──gyro_bridge──→ Insta host_ts (±1-5 ms)
                                    │
                            + sync_offset (±1-2 ms)
                                    ↓
                    Mid360 host_ts (traj_lidar.txt 查询用)
```

- **frame_0_host_ns 精度**：median SE ±0.25 ms + L_usb 常偏 1-5 ms = 走路 1-5 mm 位置偏差
- **sync_offset 精度**：Kabsch 三分量，conf=0.996+ 时 1-2 ms
- **总 pose 时间偏差 ≤ 10 ms**（走路 1 cm 位置偏差）→ 完全满足 AR 定位地图下游需求

---

## 关键陷阱

### 1. Insta cam_ts 和 fft_ts 单位不同

Insta X4 fw v1.9+:
- gyro `timestamp_ms` 单位 **ms**
- `first_frame_timestamp` 单位 **μs**

同一相机内钟，两个字段**单位差 1000 倍**。`patch_session_with_gyro_bridge.py` 用两个独立乘数处理（`cam_to_ns` 和 `fft_cam_to_ns`）。

### 2. panorama.mp4 PTS 严格 29.97 均匀化

Insta .insv 本身容器 PTS 就是严格等差 1/29.97s，不反映真实曝光 jitter。但对走路场景影响 < 几 ms，可忽略。

### 3. T_lidar_imu（Mid360 内置 IMU 外参）不用 nominal

实测用 `T_lidar_imu = [0,0,0]` + `imu_acc_noise=0.02` 比 Livox 出厂 nominal `[-0.011,-0.023,0.044]` 更稳。配 GLIM `DISPLACEMENT + 0.5m` keyframe 策略，走路场景 "IMU prediction is not good" warning 清零。见 `codeup/helmet/PANORAMA_COLORIZE.md` 步骤 1 推荐参数。

### 4. T_lidar_camera 外参精度上限 1-3 cm

dvlc 典型平移精度 1-3 cm。这是后续**所有下游（SfM, 彩色上色, VPS 定位）精度的硬天花板**。想要更高需要换标定方法（PnP + CCTag 板 / 自制高精度 jig）。

---

## 相关仓库

- **`codeup/glim-dense/`** — GLIM 稠密 raw-dump plugin + 下游 hera-desktop 几何/RGB 着色 operator 文档
- **`codeup/helmet/`**（旧工作目录，`panorama_to_prior_poses.py` 等脚本的历史副本） — 同名文件应**以本仓库为准**，`helmet/` 下的 `PANORAMA_COLORIZE.md` 是全景上色端到端操作手册
- **`xreal_metaloc/desktop-app/`** — hera-desktop UI + operator 平台（本仓库产的 prior_poses 和 T_lc 是它的输入）

---

## License

MIT（见 `LICENSE`）
