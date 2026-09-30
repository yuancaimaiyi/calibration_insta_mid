# Vistara 头盔 · VPS 定位地图输入流水线

**目标**：从 Vistara 头盔一次采集，产出可直接喂给 VPS 建图的 `每帧 ERP + prior pose` 数据。

**适用前提**：`T_lidar_camera` 已由**标定板方式**离线标定，随设备绑定出厂（一次标定长期复用）。

---

## TL;DR

采集完跑两条命令，出 `frames/*.jpg` + `prior_poses.txt`，直接喂 VPS 建图。

```bash
python3 patch_session_with_gyro_bridge.py --hera X.hera --insv X.insv --session X.session.json
python3 panorama_to_prior_poses.py --panorama panorama.mp4 --session X.session.json \
    --traj traj_lidar.txt --offset X_sync_offset.txt --extrinsic_file T_lc.txt \
    --target_fps 5 --out_dir prior_pose_out
```

---

## 硬件 & 世界

| 组件 | 型号 | 备注 |
|---|---|---|
| 全景相机 | Insta360 X4 | 竖装，ERP 输出 2:1|
| LiDAR | Livox Mid360 | 明显前倾 20-30°，含内置 IMU |
| 载体 | 白色安全头盔 + 电池腰挂 | 刚性支架把两 sensor 共装头顶 |
| 采集控制 | web 端（手机浏览器 → hera-daemon） | 一键启停两个 sensor |

**关键**：Insta360 和 Mid360 **刚性共装**，`T_lidar_camera` 一次标定长期复用；只有拆装/撞击/明显跌落才需要重标。

---

## 数据流总览

```
【首次一次性: 外参标定】(见"首次外参标定"章)
5-10 组静止扫描 (.hera+.insv+.session.json 各一份)
    │
    ├─ hera-desktop 转 db3
    ├─ hera-desktop 拼 panorama.mp4
    │
    └─ hera_to_rosbag.py × N 组 → 每组一个 bag
                ↓
      Docker: direct_visual_lidar_calibration
                ↓
      calib.json → T_lidar_camera.txt (绑定设备出厂)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【每次采集: 出 VPS 建图输入】
hera-daemon → .hera + .insv + session.json
                │
                ├─ GLIM ────→ traj_lidar.txt (host 秒)
                │
                ├─ Insta SDK 拼接 ────→ panorama.mp4
                │
                ├─ multi_source_synchronizer ────→ sync_offset.txt
                │
                └─ patch_session_with_gyro_bridge.py ⭐
                       ↓ 补 frame_0_host_ns 到 session.json
                                                          │
                                    出厂 T_lidar_camera.txt (来自首次标定)
                                                          │
                                                          ↓
                        panorama_to_prior_poses.py --extrinsic_file T_lc.txt
                                                          ↓
                            frames/<host_ns>.jpg + prior_poses.txt (TUM)
                                                          ↓
                            下游 VPS 建图（MegaLoc + ALIKED + LightGlue + 4DoF PnP）
```

---

## 环境依赖（首次部署）

### 系统层

```bash
# recorder 编译产物 (需 feat/extract-insta-packet-host-ts 分支之后)
sudo apt install libturbojpeg0-dev
cd recorder && bash build_all.sh
sudo cp build_amd64/storage/hera-storage-extract-insta /usr/local/bin/

# 验证 CSV header 是新格式
hera-storage-extract-insta some.hera --gyro /tmp/t.csv
head -1 /tmp/t.csv
# 应输出: packet_host_ns,timestamp_ms,ax,ay,az,gx,gy,gz  ← 8 列
```

### Python 层

```bash
pip install telemetry-parser numpy scipy matplotlib
# 首次标定时才需要（每次采集不用）:
pip install rosbags opencv-python
```

### 首次标定 Docker

```bash
docker pull koide3/direct_visual_lidar_calibration:humble
```

### 出厂交付文件（跟设备绑定）

- `T_lidar_camera.txt`：单行 `tx ty tz qx qy qz qw`，一次标定长期复用

---

## 首次外参标定 (T_lidar_camera) — 一次性

**触发时机**（下述之一）：
- 新头盔首次组装
- 拆装 / 撞击 / 明显跌落后
- 换 Insta360 X4 或 Mid360 单个 sensor

**方案**：无标定板，用 [`koide3/direct_visual_lidar_calibration`](https://github.com/koide3/direct_visual_lidar_calibration)（MIT 许可，Atlas Scanner 同款）—— 通过 LiDAR 强度图 ↔ ERP 全景图的 SuperGlue 特征匹配 + 图优化求 T_lc。

**精度**：0.5° / 1-2cm（indoor 良好纹理场景，与标定板方案同档）。

### Step A · 静止扫描采集（5-10 个点位）

**操作方式**（跟日常 VPS 采集完全一样，只是要求静止 + 多次）：

1. 手机 web 端点录制 button
2. 头盔戴稳，**保持静止 5-10 秒**（不摇头、不走）
3. 点停止，等 15-20 秒下载 `.insv`
4. **换个点位**（挪 2-5 m 到场景另一角度），重复 1-3
5. **总共采 5-10 个点位**

**点位选择原则**：
- 场景**有丰富纹理**（家具/柱子/地砖/管道，避开纯白墙全景）
- 每个点位**看到不同的场景内容**（视角差异越大越好）
- **室内 4x4m 以上空间**，或户外遮挡少的地方

**产出**：5-10 组 `<stem_i>.hera + <stem_i>.insv + <stem_i>.session.json`。

### Step B · 每份数据预处理

对**每一份**采集（i = 1…N）：

```bash
STEM=<stem_i>   # 换成对应 basename

# B1. gyro bridge 补 frame_0_host_ns 到 session.json
python3 patch_session_with_gyro_bridge.py \
    --hera    ${STEM}.hera \
    --insv    ${STEM}.insv \
    --session ${STEM}.session.json

# B2. hera-desktop 把 .hera 转 ROS2 bag (含 mid360 topics)
#     （产物: <stem_i>/out.db3 + metadata.yaml）
hera-desktop ...   # 按你 desktop app 里的转换流程

# B3. Insta SDK 或 hera-desktop 把 .insv 拼 panorama.mp4
#     （产物: <stem_i>/panorama.mp4）
```

### Step C · 打包成标定 bag（每份→一个 bag）

```bash
CALIB_DIR=/data/vistara_calib_bags        # 所有 bag 的父目录
mkdir -p $CALIB_DIR

for i in $(seq -f "%02g" 1 10); do   # 用 01 02 ... 10 补零, 保证字典序
    STEM=scan_${i}                    # 每次采集的 basename
    python3 hera_to_rosbag.py \
        --hera_db3   ./${STEM}/out.db3 \
        --panorama   ./${STEM}/panorama.mp4 \
        --session    ./${STEM}.session.json \
        --offset     ./${STEM}_sync_offset.txt \
        --output_bag $CALIB_DIR/${STEM}_bag \
        --target_fps 0.5    # 静止扫描，每 2s 抽一帧就够
done
```

**产出目录结构**（direct_visual_lidar_calibration 用 `std::filesystem::directory_iterator`
遍历 `$CALIB_DIR` 的直接子项，通过 `rosbag2_cpp::Reader::open()` 校验合法性，
**无文件名要求**；`--bag_id` 走字典序索引）：
```
$CALIB_DIR/                    ← 传给 preprocess_ros2 的 data_path
├─ scan_01_bag/                ← 子目录名任意, 但用 01,02,... 补零保证字典序
│  ├─ metadata.yaml            ← rosbags 库自动生成
│  └─ scan_01_bag.db3          ← rosbags 库自动生成
├─ scan_02_bag/
├─ ...
└─ scan_10_bag/
```

### Step D · Docker 内跑 direct_visual_lidar_calibration

```bash
# D1. 拉 Docker 镜像（一次即可）
docker pull koide3/direct_visual_lidar_calibration:humble

# D2. 挂载数据目录跑完整 pipeline
docker run --rm -it \
    -v /data:/data \
    koide3/direct_visual_lidar_calibration:humble bash -c "
      # 预处理：ERP 相机模型，自动扫描目录内所有 bag
      ros2 run direct_visual_lidar_calibration preprocess_ros2 \
          /data/vistara_calib_bags \
          /data/vistara_calib_out \
          --auto_topic \
          --camera_model equirectangular \
          --image_topic /insta/image \
          --points_topic /lidar/mid360/point_cloud2 \
          --camera_info_topic /insta/camera_info \
          --dynamic_lidar_integration && \

      # 初始外参（SuperGlue 匹配 LiDAR 强度图 ↔ ERP 图）
      ros2 run direct_visual_lidar_calibration find_matches_superglue.py \
          /data/vistara_calib_out --rotate_camera 0 && \
      ros2 run direct_visual_lidar_calibration initial_guess_auto \
          /data/vistara_calib_out && \

      # 精化
      ros2 run direct_visual_lidar_calibration calibrate \
          /data/vistara_calib_out
    "
```

**产出**：`/data/vistara_calib_out/calib.json`，内含：
```json
{
  "T_lidar_camera": [x, y, z, qx, qy, qz, qw]
}
```

### Step E · 转成本 pipeline 的 T_lc.txt 格式

`panorama_to_prior_poses.py --extrinsic_file` 期望一行 7 个数 `tx ty tz qx qy qz qw`。calib.json 里的 `T_lidar_camera` 数组顺序一致，只需转文本：

```bash
python3 -c "
import json
d = json.load(open('/data/vistara_calib_out/calib.json'))
t = d['T_lidar_camera']
print(' '.join(f'{v:.9f}' for v in t))
" > T_lidar_camera.txt
```

**这就是你出厂随头盔交付的 T_lc.txt**，一次标定终生复用。

### Step F · 视觉验证（关键）

**有 GUI 的机器（推荐）**：Iridescence 交互式 3D+2D 窗口

```bash
docker run --rm -it \
    --network=host \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    -v /data:/data \
    koide3/direct_visual_lidar_calibration:humble \
    ros2 run direct_visual_lidar_calibration viewer /data/vistara_calib_out
```

窗口能看到：
- **3D LiDAR 点云场景**（鼠标可转视角）
- **每张相机图 + LiDAR 点云叠加投影**（⭐ 最关键的图）
- **SuperGlue 每对特征匹配连线**
- **NID cost 优化收敛曲线**

**无 GUI 的机器**（服务器场景）：pipeline 自动 dump PNG

```bash
ls /data/vistara_calib_out/*.png       # 每个 scan 一张 overlay 图
# 拷到本地看:  scp remote:/data/vistara_calib_out/*.png .
```

**"读图"标准**（看每张 overlay 图）：

| 距离档 | 点云 vs 图像边缘 | 判定 |
|---|---|---|
| **近景 1-2 m**（桌腿、椅背） | 应贴 < 5 px | ✅ 好 / ❌ T_lc 平移错 |
| **中距 3-5 m**（门框、柱子） | 应对齐 | ✅ 好 |
| **远景 > 10 m**（天花板远端） | 允许几个像素偏移（LiDAR 稀疏正常） | 忽略 |

**症状 → 原因**：
- 所有距离都不贴，但都朝**同一方向**偏 → **T_lc 旋转错**
- 近处贴、远处飘 → **T_lc 平移错**（近处平移影响像素多）
- 局部贴、局部飘 → 通常没大问题，SuperGlue 匹配噪声

### 标定失败常见原因

| 症状 | 根因 | 处理 |
|---|---|---|
| `SuperGlue matches < 10` | ERP 图特征太少 | 采集点位换有更多纹理的场景 |
| `initial_guess_auto` 报错 | LiDAR 视场没覆盖相机主要视角 | 让 Mid360 dome 主方向对着有结构的地方 |
| viewer 里点云严重偏移 | 采集点位太单一（视角相似） | 增加点位数，让每个点位场景差异大 |
| `--rotate_camera` 试了 0/90/180/270 都不行 | ERP 定义方向跟 Mid360 主方向差异复杂 | 联系我调试 |

### 快速判据

标定跑完看 calibrate 输出的：
- `nid`（Normalized Information Distance）**< 0.7** → 良好
- `nid` 0.7-0.85 → 可用
- `nid > 0.9` → 失败，需重采

---

## 完整流水线（每次采集）

### Step 0 · 采集

hera-daemon（web 端手机浏览器）一键启动。产出：

| 文件 | 内容 |
|---|---|
| `<stem>.hera` | Mid360 点云 + IMU + Insta 陀螺仪（单文件） |
| `<stem>.insv` | Insta 双鱼眼原始视频 |
| `<stem>.session.json` | `record_start_host_ns` 等 metadata |

**采集动作要求**（决定后续精度）：

- 前 60s：**S 形走 + 摇头 + 蹲起**（给 sync 相关性 & 未来重新标定的 6DoF 激励用）
- 后段：**匀速走**，避免速度突变
- 总时长 60-120s

### Step 1 · GLIM 出激光轨迹

```
.hera → GLIM (含正确 T_lidar_imu 出厂值) → traj_lidar.txt
```

**关键 T_lidar_imu 出厂固定值**（Livox Mid360，GLIM 配置必须写对）：
```
translation = [-0.011, -0.02329, 0.04412] m
rotation    = identity (0, 0, 0, 1)
```

**输出**：`traj_lidar.txt`，第一列 host 绝对秒（~1.79e9），z-up gravity-aligned world。

### Step 2 · 全景拼接

```
.insv → Insta SDK (MediaSDKTest / hera-desktop) → panorama.mp4
      ERP 5760×2880, PTS 从 0 起
```

**注意**：如果做了任何剪辑（`ffmpeg -ss` 之类），会破坏时间锚点对应关系。**保持拼接产物原样**。

### Step 3 · 时间同步

```
.hera → multi_source_synchronizer → <stem>_sync_offset.txt
```

**输出字段**：`offset_sec`，语义 `t_mid = t_insta + offset_sec`（Mid ↔ Insta 两侧回调时延差，通常 ±100ms）。

**采集动作不达标**（前 60s 无大幅晃动） → sync 可能无解，脚本会走 mock=0 兜底，接受 ±100~500ms 系统误差（下游 prior 精度打折）。

### Step 3.5 · 视频时间锚点修正 ⭐

**问题**：`record_start_host_ns` 是用户点采集 button 时的 host 时刻（SDK 异步返回），比相机真正曝光第 0 帧**早几百 ms 到上千 ms**（USB 传输 + 相机 sensor startup 延迟），且**每次开机偏差不同**，不能用固定常数补偿。

**方案**：Gyro Clock Bridge —— 从 `.hera` gyro packet 双时戳 + `.insv` `first_frame_timestamp` 反推真实第 0 帧曝光时刻。

```bash
python3 patch_session_with_gyro_bridge.py \
    --hera    <stem>.hera \
    --insv    <stem>.insv \
    --session <stem>.session.json \
    [--diag]              # 可选：打印 ASCII 直方图 + 时间漂移分析
    [--plot offset.png]   # 可选：输出诊断 PNG
```

**产出**：`session.json` 新增 `frame_0_host_ns` 字段，精度 **~1 ms**。

**详细原理见** `VIDEO_FRAME_0_HOST_TIMESTAMP.md`。

### Step 4 · 抽帧 + 打 pose

```bash
python3 panorama_to_prior_poses.py \
    --panorama       panorama.mp4 \
    --session        <stem>.session.json \
    --traj           traj_lidar.txt \
    --offset         <stem>_sync_offset.txt \
    --extrinsic_file <设备出厂 T_lidar_camera.txt> \
    --target_fps     5 \
    --out_dir        prior_pose_out
```

**参数说明**：

| 参数 | 说明 |
|---|---|
| `--target_fps 5` | 视频下采样到 5 fps。VPS 建图不需要 30fps，5fps 足够且省算力。CLI 支持 `--max_frames N` 换成"总帧数上限"模式 |
| `--extrinsic_file` | 出厂标定 T_lc（标定板产物） |
| 其他 | 默认值均合理 |

**内部动作**：
1. 读 `frame_0_host_ns`（三层优先级：gyro_bridge > gps > record_start）
2. 从 mp4 PTS 算每帧 host_sec = `frame_0_host_ns/1e9 + PTS + offset_sec`
3. 按 host_sec 命名图像 `<host_ns>.jpg`（时间戳固化进文件名）
4. `--target_fps` 时间等间隔抽样
5. 对每帧 host_sec 在 `traj_lidar.txt` 上 Slerp+linear 插值 → `T_world_lidar(t)`
6. 拼 `T_world_camera(t) = T_world_lidar(t) @ T_lidar_camera`
7. 输出 TUM 格式 prior_poses.txt

---

## 最终产出（喂 VPS 建图的两样东西）

```
prior_pose_out/
├─ frames/
│  ├─ <host_ns_1>.jpg          # ERP 5760×2880，文件名 = 该帧曝光的 host 纳秒
│  ├─ <host_ns_2>.jpg
│  └─ ...
├─ frame_timestamps.csv        # frame_index, image_name, t_host_sec
├─ prior_poses.txt             # ⭐ TUM 格式，每帧 T_world_camera
└─ extrinsic_sim.txt           # 仿真占位外参（本流程用了 --extrinsic_file，此文件无实际影响）
```

**`prior_poses.txt` 格式**：
```
# image_name tx ty tz qx qy qz qw
# T_world_camera = T_world_lidar(t_host) @ T_lidar_camera
# time_base = absolute host seconds (aligned with traj_lidar.txt)
1789992760234405776.jpg 0.123 0.456 1.789 -0.354 0.354 -0.612 0.612
...
```

**坐标系约定**：
- `(tx, ty, tz)`：相机光心在 **z-up gravity-aligned world**（LiDAR world）下的位置
- `(qx, qy, qz, qw)`：相机 local (OpenCV: x 右 / y 下 / z 前) → world 的旋转

**下游直接吃**：COLMAP / HLoc / VPS 4-DoF 重力约束 PnP 全兼容。

---

## 精度预估

| 环节 | 精度贡献 | 备注 |
|---|---|---|
| T_lc（标定板） | ~mm | 出厂离线标定 |
| frame_0_host_ns（gyro bridge） | ~1 ms | 走路 1m/s 换算 → 1 mm 位置误差 |
| sync_offset（gyro 互相关） | ~5-10 ms | 走路 → 5-10 mm 位置误差 |
| GLIM 定位精度 | ~2-5 cm | LiDAR-inertial SLAM 典型 |
| **每帧 prior pose 综合精度** | **~3-6 cm 位置 / <1° 姿态** | 对 VPS 建图（dm 级）绰绰有余 |

---

## 验证清单（每次跑完必看）

### patch 阶段（`patch_session_with_gyro_bridge.py`）

| 输出行 | 健康值 |
|---|---|
| `[bridge] gyro packets = N` | **N > 500**（100s 录制约 1000） |
| `[bridge] gyro cam unit = ms` | 必须是 `ms`（sample_diff_median ≈ 2） |
| `[bridge] offset std = X ms` | **X < 10 ms** |
| `[chk] frame_0 - record_start = +X ms` | **X 应为正值**（几百 ms 到 ~1500 ms） |

### 抽帧阶段（`panorama_to_prior_poses.py`）

| 输出行 | 健康值 |
|---|---|
| `[t0] video anchor host_ns = ... (first_gps_timestamp_ns/frame_0_host_ns/...)` | **必须显示 gyro_bridge**（用了 patch），如显示 `record_start_host_ns` → patch 步骤没跑 |
| `[traj] N poses, span Xs` | 与录制时长一致 |
| `[pts] overlap window (Xs)` | 通常 = LiDAR traj span |
| `[img] kept M/N frames` | 与 target_fps × overlap_span 一致 |
| `[pose] wrote M, skipped 0` | 全部帧成功赋 pose |

### 数据文件

- `frames/*.jpg` 数量 = `[img] kept M` 输出
- `prior_poses.txt` 行数 = `[pose] wrote M`
- 每张图文件名前缀（host_ns）单调递增

---

## 常见故障排查

### 问题 1：`patch` 报 gyro packet 0 或 CSV 缺列

**症状**：patch 脚本报错 `CSV 里没有 timestamp_ms 列` 或 `gyro packets < 5`

**原因**：`hera-storage-extract-insta` 是老版本

**处理**：切到 recorder `feat/extract-insta-packet-host-ts` 分支重编 + 安装

### 问题 2：`frame_0 - record_start` 为负数

**症状**：`[chk] frame_0 - record_start = -X ms`

**原因**：
1. `first_frame_timestamp` 单位错（本 pipeline μs，Insta X4 v1.9+ 约定；老固件可能是 ms）
2. gyro sample.timestamp 单位错（本 pipeline ms）

**处理**：`--fft_unit_multiplier 1000000` 换成 ms，或跟 Insta 固件版本核对

### 问题 3：`sync_offset.txt` 无 offset

**症状**：sync 文件里 `confidence=0` 或没有 `offset_sec=` 字段

**原因**：采集前 60s 未做大幅晃动

**处理**：
- 立刻可用：`--mock_offset_zero` 跳过，接受 ±100~500ms 系统偏
- 长期修复：下次采集严格执行"前 60s S形+摇头+蹲起"

### 问题 4：panorama.mp4 首帧 PTS 不是 0

**症状**：ffprobe 显示 mp4 首帧 PTS > 0

**原因**：
1. 拼接后被 ffmpeg 剪辑了（`-ss` 参数）
2. 用了 gyro SDK 输出（首帧从 77s 起是相机内钟，不是 mp4 PTS）

**处理**：**永远用 hera-desktop / MediaSDKTest 的原始拼接产物**，别做任何剪辑

### 问题 5：VPS 建图完精度差

**症状**：VPS 定位偏 dm 级或更大

**排查顺序**：
1. patch 输出的 offset std < 10ms？（不是 → USB 抖动大，检查采集环境）
2. `frame_0 - record_start` 是正值？（不是 → 单位错）
3. `T_lc.txt` 是这台机器的？（不是 → 拿错设备）
4. traj_lidar.txt 完整覆盖录制时长？（不是 → GLIM 掉帧）
5. prior_poses.txt 里 tz 是否合理（不飘）？（否 → GLIM 长程 drift 或标定 z 分量塌陷）

---

## 出厂交付 & 用户使用

**出厂随头盔交付**：
- `T_lidar_camera.txt`（一机一份，由本文档"首次外参标定"章 direct_visual_lidar_calibration 产出）
- 本文档 + `patch_session_with_gyro_bridge.py` + `panorama_to_prior_poses.py` + 依赖清单

**终端用户体验**：
1. 头盔戴上，web 端点录制按钮
2. 按采集动作要求走 60-120 秒
3. 点停止，等 15-20 秒下载 .insv
4. **两条命令跑完** → prior pose 数据就绪
5. 交给 VPS 建图工具

**无需接触**：direct_visual_lidar_calibration、Docker、hera_to_rosbag —— 这些只在**首次标定**（新头盔组装 / 拆装 / 撞击后）用到，出厂后终端用户不接触。

---

## 相关文档

- **`PIPELINE.md`** —— 完整版 pipeline（含 SfM+CCTag 自标外参分支，早期方案）
- **`VIDEO_FRAME_0_HOST_TIMESTAMP.md`** —— gyro bridge 原理详解 & 判据 & Q&A
- **`TIME_SYNC_AND_FRAME_TIMESTAMP.md`** —— 时基锚点数学细节
- **`patch_session_with_gyro_bridge.py`** —— gyro bridge 实现（含 `--diag/--plot`）
- **`panorama_to_prior_poses.py`** —— 抽帧 + prior pose 主脚本
- **`hera_to_rosbag.py`** —— 首次标定用，把 (hera-desktop 转的 db3 + panorama.mp4) 合成一个 ROS2 bag 喂 direct_visual_lidar_calibration

## 一句话总结

**Vistara 出厂后每次采集只做两步**：patch 时间锚点 + 出 prior pose。剩下所有硬活（标定 / 时间同步机制 / gyro bridge 数学）都已一次性做好，用户无感。
