# アームロボットの学習

CRANE+ アームが Gazebo 上で箱をつかみ，おもちゃの車の屋根に置くタスクを，カメラ画像から模倣学習（ACT / SmolVLA）で学習するプロジェクトです．教師データは MoveIt で動くスクリプトのエキスパートが作ります．参考文献[1]の手順を元にしています．

```
randomize_box.py   → datasets/<ep>/initial_pose.json   箱と車の位置をランダムに決める
collect_dataset.py → datasets/<ep>/bag, result.json    シーンを置き，エキスパートを動かして録画
convert2npz.py     → datasets/<ep>/episode.npz         10 Hz にリサンプル
learning/build_dataset.py → lerobot_data/              LeRobotDataset (v3.0) に変換
lerobot-train                                          ACT / SmolVLA を学習
learning/policy_server.py + scripts/infer_node.py      Gazebo で推論
```

## 1. Docker イメージとコンテナ
筆者は学校のネットワーク（Proxy あり）で開発したため，Proxy と GPU を設定したスクリプトを使います．学校のネットワークでない場合は `--build-arg` の2行と `docker_start.ps1` の Proxy 設定を外してください．

イメージには，元のイメージに加えて次のものが入ります．
- `ros-humble-rosbag2-storage-mcap`（録画用）
- `/opt/lerobot-venv`：`learning/uv.lock` どおりの LeRobot 0.6.1 環境（Python 3.13，`policy_server.py` 専用）．ROS Humble は Python 3.10 のため，LeRobot（3.12 以上が必要）とは別の Python にしています．
- `pyzmq`（システムの Python 3.10，`infer_node.py` 用）

ビルドはリポジトリのルートで行います（`.dockerignore` で `learning/pyproject.toml` と `learning/uv.lock` だけを送ります）．
```powershell
docker build -t crane-arm:humble -f docker/Dockerfile `
  --build-arg HTTP_PROXY=http://wproxy.net.sendai-nct.ac.jp:8080 `
  --build-arg HTTPS_PROXY=http://wproxy.net.sendai-nct.ac.jp:8080 `
  .
```
同じ名前のコンテナが既にある場合は，先に削除してから起動します（リポジトリはマウントなので消えません．起動後に `colcon build` をやり直してください）．
```powershell
docker rm -f custom-crane
./scripts/docker_start.ps1
```
Terminal は必ず Windows の Terminal を使ってください（VSCode 内では Gazebo がうまく表示されないことがあります）．
```bash
docker exec -it -u ubuntu -e DISPLAY=:0 custom-crane bash
cd ~/airobot_ws/src/user_code
```
イメージの確認（GPU が見えていれば `True`）．ベースイメージの entrypoint がデスクトップを起動するため，`--entrypoint` で直接実行します．
```powershell
docker run --rm --gpus all --entrypoint /opt/lerobot-venv/bin/python crane-arm:humble -c "import torch, lerobot, zmq; print(torch.cuda.is_available())"
```

## 2. シミュレーション
Gazebo（カメラ付きの机と車）と3台のカメラのブリッジを起動します．Ctrl+C で止めます．
```bash
./scripts/run_simulation.sh
ros2 topic hz /camera_top/image_raw   # 約 10 Hz なら OK
```
エキスパートのビルドと単体実行：
```bash
cd ~/airobot_ws
colcon build --symlink-install --packages-select crane_dataset_expert
source install/setup.bash
ros2 launch crane_dataset_expert dataset_expert.launch.py                      # scene.yaml の既定位置
EPISODE=<episode> ros2 launch crane_dataset_expert dataset_expert.launch.py    # datasets/<episode> の位置
```

## 3. データ収集
1. シーンを作る：`python3 scripts/randomize_box.py --seed <n> --episode <name>`（`config/scene.yaml` の範囲で箱と車を置き，`datasets/<name>/initial_pose.json` を書きます）．
2. 録画する（シミュレーション起動中）：`python3 scripts/collect_dataset.py`．`result.json` がないエピソードだけを順に録画します．1エピソード約1分．止めても再実行で続きから再開します．
   - `/joint_states` と3台のカメラ（320×240）を mcap で録画します．
   - 終了後に箱の最終位置を Gazebo から読み，屋根の中心から 3 cm 以内かつ屋根の高さ ±1 cm なら `"success": true` を `result.json` に書きます．
3. 変換する：`python3 scripts/convert2npz.py`．`expert_exit` が 0 で `success` が false でないエピソードを 10 Hz の `episode.npz` にします（`action` は 0.1 秒後の関節角）．

1エピソードは bag 約 300 MB + npz 約 95 MB です．ディスクの空きに注意してください．

## 4. 学習（Linux / WSL，`learning/` の uv 環境）
LeRobot の画像書き出しは `/mnt/c` だと非常に遅いので，Linux 側のディスクにコピーして行います．
```bash
mkdir -p ~/arm && cd ~/arm
rsync -a --exclude .venv /mnt/c/Users/pongs/Documents/GitHub/Arm-Robot-Picking/learning ./
rsync -a --include='*/' --include='episode.npz' --exclude='*' \
  /mnt/c/Users/pongs/Documents/GitHub/Arm-Robot-Picking/datasets ./
cd learning && uv sync
rm -rf ../lerobot_data && uv run build_dataset.py   # ~/arm/lerobot_data/crane_pick_place
bash train.sh                                        # ACT
```
SmolVLA は `smolvla_base` から fine-tune します（既定では視覚エンコーダと言語モデルは凍結され，action expert だけを学習します）．カメラ名は `--rename_map` で合わせます．
```bash
uv run lerobot-train --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=local/crane_pick_place --dataset.root=$HOME/arm/lerobot_data/crane_pick_place \
  --dataset.video_backend=pyav --policy.device=cuda --policy.push_to_hub=false \
  --rename_map='{"observation.images.top": "observation.images.camera1", "observation.images.left": "observation.images.camera2", "observation.images.right": "observation.images.camera3"}' \
  --batch_size=32 --steps=30000 --save_freq=5000 --output_dir=$HOME/arm/outputs/smolvla_crane
```
オフライン評価（データセットの観測を方策に入れ，エキスパートの行動と比べます）．引数にエピソード番号，または学習に使っていない `episode.npz` のパスを渡します．
```bash
uv run eval_offline.py ~/arm/outputs/act_crane/checkpoints/last/pretrained_model 0
```

## 5. 推論（コンテナ内，Gazebo）
学習済みの `pretrained_model` フォルダを `learning/checkpoints/<name>/` に，データセットの `meta/` を `lerobot_data/crane_pick_place/meta/` に置きます（どちらも git には含めません）．
```bash
# Terminal 1
./scripts/run_simulation.sh
# Terminal 2：方策サーバ（Python 3.13，GPU）
/opt/lerobot-venv/bin/python learning/policy_server.py learning/checkpoints/<name>
# Terminal 3：ROS ノード（Python 3.10）．/joint_states と3台のカメラを送り，返ってきた行動を
#             /crane_plus_arm_controller/joint_trajectory と /crane_plus_gripper_controller/joint_trajectory に publish
python3 scripts/infer_node.py
```
2つのプロセスはコンテナ内の `tcp://127.0.0.1:5555`（ZMQ）で通信します．

## 6. 現在の結果
- **データ**：206 シーンをサンプリングし，41 エピソードを録画（エキスパートは 41/41 完了．屋根判定を入れた後の 9 エピソードは 9/9 成功）．学習には最初の 32 エピソードを使用．
- **ACT**（32 エピソード，20k steps，batch 8，chunk 50）：loss 7.23 → 0.067，l1_loss 0.563 → 0.057．
- **オフライン評価**（学習に使ったエピソード 20 本）：平均誤差は各関節の動作範囲の約 1%．ただしグリッパの最大誤差は毎回ほぼ全範囲（約 0.5 rad）で，把持の瞬間を外しています．
- **「1つ前の行動をそのまま返す」ベースラインとの比較**（エピソード 2）：方策の平均誤差は全関節でベースラインより大きい（グリッパは 0.0297 vs 0.0053）．ただし `n_action_steps=50` のまま評価しているため，`n_action_steps=1` での再評価が必要です．
- **Gazebo での推論**：箱の位置をとらえられず，把持・配置に失敗．

### 考えられる原因と次の手順
1. `action = 0.1 秒後の関節角` は現在の状態とほぼ同じなので，方策が画像を見ずに状態をコピーするだけで loss が下がる（causal confusion / copycat）．→ コントローラへの指令値か数ステップ先の関節角を行動にし，動いていないフレームを除く．
2. 箱と車の位置が毎回変わるのに，32 エピソードでは少ない．→ 206 シーンを録画し，50 シーンをテスト専用に分ける．
3. `n_action_steps=50` だと 5 秒間画像を見ずに動く．→ 推論では 1〜10 にする．
4. 目標：テスト 50 シーンで ACT の成功率 50% 以上（屋根判定で計測）．その後，箱の位置の入力（YOLO），SmolVLA，DAgger を1つずつ比較する．

## 便利なコマンド
```bash
# 箱を既定の位置に戻す
ign service -s /world/default/set_pose \
  --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 \
  --req 'name: "pick_box", position: {x: 0, y:-0.1, z:1.015}, orientation: {w: 1}'

# カメラ画像を見る
export DISPLAY=:0
ros2 run rqt_image_view rqt_image_view

# ロボットの初期位置
ros2 run tf2_ros tf2_echo world base_link

# モデルの位置
ign model -m placement_car -p
ign model -m pick_box -p
for m in $(ign model --list | grep -- '- ' | sed 's/.*- //'); do
  echo "== $m"; ign model -m "$m" -p | sed -n '/Pose/,+2p'
done
```

## 参考文献

1. [ROS 2 講義資料](https://skinny-chord-c58.notion.site/5-ROS-2-2-2-2049a7a1061380fc8928e500eafc57d0)
2. [matsuolab/pai_training](https://github.com/matsuolab/pai_training) — 講義の学習パイプライン．関節角のみの ACT．
3. [LeRobot](https://github.com/huggingface/lerobot)
4. Zhao et al., "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware," RSS 2023.
5. Shukor et al., "SmolVLA: A Vision-Language-Action Model for Affordable and Efficient Robotics," 2025.
6. Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion," RSS 2023.
7. Mandlekar et al., "What Matters in Learning from Offline Human Demonstrations for Robot Manipulation," CoRL 2021.
8. de Haan et al., "Causal Confusion in Imitation Learning," NeurIPS 2019.
9. Wen et al., "Fighting Copycat Agents in Behavioral Cloning from Observation Histories," NeurIPS 2020.
10. Ross et al., "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning," AISTATS 2011.
