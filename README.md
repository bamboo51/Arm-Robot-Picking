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