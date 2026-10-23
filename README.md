# アームロボットの学習

## はじめに
まず，参考文献[1]に基づいて，アームロボットの実装まで進みます．

筆者が開発したときに，学校のインターネット環境下に開発したため，ProxyとGPUを設定したscriptsを実装します．
```powershell
./scripts/docker_start.ps1
```

使うTerminalは，必ずWindowsのTerminalを使ってください．VSCode内で実装すると，Gazeboがうまく出力できない可能性があります．
次は，DockerのTerminalを開きます．
```bash
docker exec -it -u ubuntu -e DISPLAY=:0 pai_ros2 bash
```

TerminalAでは，moveitを入れないGazeoを実装します．
```bash
source ~/airobot_ws/install/setup.bash

ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
```
TerminalBでは，Docker内のTerminalで例を実装します．
```bash
source ~/airobot_ws/install/setup.bash

ros2 launch crane_plus_examples \
  example.launch.py \
  example:=pick_and_place \
  use_sim_time:=true
```

Boxを元の位置に戻るためには，
```bash
ign service -s /world/default/set_pose \
  --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 \
  --req 'name: "pick_box", position: {x: 0, y:-0.1, z:1.015}, orientation: {w: 1}'
```

## カメラに関するコマンド
```bash
# TerminalCでカメラブリッジ
ros2 run ros_gz_bridge parameter_bridge \
    '/camera_left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image' \
    '/camera_left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo' \
    '/camera_right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image' \
    '/camera_right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo' \
    '/camera_top/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image' \
    '/camera_top/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'

# TerminalDでカメラを確認
export DISPLAY=:0
ros2 run rqt_image_view rqt_image_view
```

## ロボットの初期位置の確認コマンド
```bash
ros2 run tf2_ros tf2_echo world base_link
```

### 学習データ収集


## 参考文献
[1] https://skinny-chord-c58.notion.site/5-ROS-2-2-2-2049a7a1061380fc8928e500eafc57d0

LeRobot: https://github.com/huggingface/lerobot. Its docs cover dataset format, ACT and SmolVLA training.
ACT paper: Zhao et al., 2023, "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware".
SmolVLA paper: Hugging Face, 2025.
Diffusion Policy: Chi et al., 2023.
DAgger: Ross et al., 2011, "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning".