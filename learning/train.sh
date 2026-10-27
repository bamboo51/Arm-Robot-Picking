cd ~/arm/learning
uv run lerobot-train \
  --dataset.repo_id=local/crane_pick_place \
  --dataset.root=$HOME/arm/lerobot_data/crane_pick_place \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --policy.chunk_size=50 \
  --policy.n_action_steps=50 \
  --batch_size=8 \
  --steps=20000 \
  --save_freq=5000 \
  --log_freq=200 \
  --num_workers=4 \
  --output_dir=$HOME/arm/outputs/act_crane \
  --job_name=act_crane \
  --wandb.enable=true \
  --wandb.project=crane \
  2>&1 | tee train.log
