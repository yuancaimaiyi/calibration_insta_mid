docker run --rm --gpus all \
    -v /home/yuancaimaiyi/feima_data/calibration:/data \
    dvlc:rgb bash -c "
      set -e
      apt-get update -qq && apt-get install -y -qq xvfb 2>&1 | tail -1

      # D1. preprocess (注意 --dynamic_lidar_integration 必需, Mid360 non-repetitive)
      ros2 run direct_visual_lidar_calibration preprocess \
          --data_path /data/bags --dst_path /data/calibration_result \
          --auto_topic --camera_model equirectangular \
          --image_topic /insta/image --points_topic /lidar/mid360/point_cloud2 \
          --camera_info_topic /insta/camera_info --dynamic_lidar_integration

      # D2. SuperGlue 匹配 (indoor weights 对室内办公场景; 户外用 outdoor)
      ros2 run direct_visual_lidar_calibration find_matches_superglue.py \
          /data/calibration_result --rotate_camera 0 --superglue indoor

      # D3. RANSAC + LSQ 出 initial guess
      ros2 run direct_visual_lidar_calibration initial_guess_auto /data/calibration_result

      # D4. NID 精化 (xvfb 绕 GLFW init bug)
      xvfb-run -a --server-args='-screen 0 1024x768x24' \
        ros2 run direct_visual_lidar_calibration calibrate /data/calibration_result --background --auto_quit

      cat /data/calibration_result/calib.json
    "
python3 -c "
import json
d = json.load(open('/path/to/calibration/out/calib.json'))
print(' '.join(f'{v:.9f}' for v in d['results']['T_lidar_camera']))
" > /home/yuancaimaiyi/feima_data/calibration/T_lidar_camera.txt
