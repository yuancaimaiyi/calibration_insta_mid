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

**一条龙脚本**：`run_calibration.sh`（最新流程，封装所有 Step）
**文档**：`calibration.md`（Dockerfile 制作 + 5 大踩坑点）

**一次性 Docker 镜像准备**：`dvlc:superglue`（base + torch + SuperGlue）和 `dvlc:rgb`（+ RGB NID 修补）—— 见 `calibration.md` 的 Dockerfile 节。

**采集**：**至少 3 站**静止扫描（纹理丰富场景，典型 5-10 站更稳）。每站用 hera-desktop 产出以下 6 个文件放到 `DATA_ROOT/<N>/` 下：

```
DATA_ROOT/
├── 1/
│   ├── <stem>.hera                 ← hera-daemon 原始录制
│   ├── <stem>.insv                 ← Insta 原始视频
│   ├── <stem>.session.json         ← hera-daemon 原始
│   ├── out.db3                     ← hera-desktop「hera_to_bag」卡片转的
│   ├── metadata.yaml               ← out.db3 配套
│   └── panorama.mp4                ← hera-desktop「全景拼接」卡片 template 算法拼的
├── 2/ ...
├── N/ ...
```

**一条龙跑**（本仓库同目录）：

```bash
./run_calibration.sh /path/to/DATA_ROOT
# 可选: --no-viewer  (远端或无 X11)
#       --docker-image dvlc:superglue  (灰度版验收)
#       --helmet-dir   ...             (自定义脚本目录)
```

**内部 Step** （脚本自动做, 无需手动）：
- **B1**: 每站 `patch_session_with_gyro_bridge.py`（打 `frame_0_host_ns` 到 session.json）
- **C**: 每站 `hera_to_rosbag.py --target_fps 0.5` → `bags/scan_XX_bag/`
- **D**: docker 跑 dvlc 四段：`preprocess (--dynamic_lidar_integration)` → `find_matches_superglue (--superglue indoor)` → `initial_guess_auto` → `calibrate (NID + xvfb-run)`
- **E**: 从 `calibration_result/calib.json` 提 7 数写 `T_lidar_camera.txt`，脚本自动做 `|t|` 量级 sanity check
- **F**: 自动打开 dvlc viewer（可 `--no-viewer` 关），同时产 SuperGlue 匹配图 `scan_XX_bag_superglue.png` 每站一张

**验收**：
- `|t|` 应在 **3-50 cm**（典型 Vistara 头盔 ~5-15 cm；<3cm 偏小、>50cm 跑飞）
- 看 `calibration_result/scan_*_bag_superglue.png` 匹配密度（>50 对良好）
- viewer 里点云投影到 ERP 边缘对齐（近景 <5 px）
- NID < 0.7 良好，0.7-0.85 可用，>0.9 失败

**出厂产物**：`DATA_ROOT/T_lidar_camera.txt`（TUM 格式：`tx ty tz qx qy qz qw`），随设备交付。

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
| **`run_calibration.sh`** | **一条龙首次标定脚本**（最新, 封装 B1→C→D→E→F 所有 Step, 推荐入口） |
| `calibration.md` | 首次标定详细文档（Dockerfile 制作 + 5 大踩坑点）|
| `VISTARA_VPS_INPUT.md` | 每次采集流程（2 命令出 prior pose）|
| `patch_session_with_gyro_bridge.py` | gyro clock bridge 算视频第 0 帧 host_ns（精度 ±1-5 ms）|
| `panorama_to_prior_poses.py` | ffmpeg 抽帧 + 按 host_ns 命名 + 查 traj_lidar 插值 T_world_camera |
| `hera_to_rosbag.py` | 把 .db3 (lidar+imu) + .mp4 (全景) 打包成 ROS2 bag (供 dvlc) |
| `calibration.sh` | 单站标定壳子脚本（run_calibration.sh 之前的旧版） |
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
