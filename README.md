# アームロボットの学習

## はじめに
まず，参考文献[1]に基づいて，アームロボットの実装まで進みます．

筆者が開発したときに，学校のインターネット環境下に開発したため，ProxyとGPUを設定したscriptsを実装します．
```powershell
./scripts/docker_start.ps1
```

### mcap入りのイメージを作る（データ収集用）
`ros2 bag record -s mcap` を使うため，元のイメージに `ros-humble-rosbag2-storage-mcap` を追加したイメージを一度だけビルドします．学校のネットワークでない場合は `--build-arg` の2行を外してください．
```powershell
docker build -t crane-arm:humble `
  --build-arg HTTP_PROXY=http://wproxy.net.sendai-nct.ac.jp:8080 `
  --build-arg HTTPS_PROXY=http://wproxy.net.sendai-nct.ac.jp:8080 `
  docker
```
起動スクリプトの最後の行のイメージ名 `airobotbook/ros2-desktop-ai-robot-book-humble` を `crane-arm:humble` に変えます．同じ名前のコンテナが既にある場合は，先に削除してから起動します（リポジトリはマウントなので消えません．起動後に `colcon build` をやり直してください）．
```powershell
docker rm -f pai_ros2_gpu
./scripts/docker_start2.ps1
```
コンテナ内で確認します（Ctrl+Cで止めて，エラーが出なければOK）．
```bash
ros2 bag record -s mcap -o /tmp/mcap_test /joint_states
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

## 世界のモデルの位置をチェック
```bash
ign model --list

# pose of one model
ign model -m placement_car -p
ign model -m pick_box -p

# pose of all model
for m in $(ign model --list | grep -- '- ' | sed 's/.*- //'); do
  echo "== $m"; ign model -m "$m" -p | sed -n '/Pose/,+2p'
done
```

## DataExpertの実装について
```bash
# ビルド
cd ~/airobot_ws
colcon build --symlink-install --packages-select crane_dataset_expert
source install/setup.bash

# 実装
ros2 launch crane_dataset_expert dataset_expert.launch.py                      # default pose
EPISODE=ep_000001 ros2 launch crane_dataset_expert dataset_expert.launch.py    # randomized
```
### 学習データ収集


## 参考文献
[1] https://skinny-chord-c58.notion.site/5-ROS-2-2-2-2049a7a1061380fc8928e500eafc57d0

LeRobot: https://github.com/huggingface/lerobot. Its docs cover dataset format, ACT and SmolVLA training.
ACT paper: Zhao et al., 2023, "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware".
SmolVLA paper: Hugging Face, 2025.
Diffusion Policy: Chi et al., 2023.
DAgger: Ross et al., 2011, "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning".