"""Replay one recorded episode through a trained policy and compare its actions to the expert's.

uv run eval_offline.py <checkpoint>/pretrained_model [episode_index | path/to/episode.npz]
An episode.npz lets you test an episode that is not in the training set.
DATA=<lerobot dataset folder> picks the dataset the policy was trained on; K=<frames ahead> sets the
target for an episode.npz (it must match how that dataset was built).
"""
import os
import sys
from pathlib import Path

import numpy as np
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies import make_policy, make_pre_post_processors
from gripper_label import hand_target

ROOT = Path(os.environ.get("DATA", Path.home() / "arm/lerobot_data/crane_pick_place")).expanduser()
K = int(os.environ.get("K", 5))
BLACK = os.environ.get("BLACK") == "1"  # ablation: black images; if the error barely changes, the policy ignores vision
RENAME = {"observation.images.top": "observation.images.camera1",
          "observation.images.left": "observation.images.camera2",
          "observation.images.right": "observation.images.camera3"}

if len(sys.argv) < 2:
    sys.exit(__doc__)
ckpt, ep = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "0"
npz = ep.endswith(".npz")
ds = LeRobotDataset(f"local/{ROOT.name}", root=ROOT, episodes=[0 if npz else int(ep)])

def frames():
    """Yield (observation, expert action) per frame, from the dataset or from a raw episode.npz."""
    if not npz:
        for i in range(len(ds)):
            f = ds[i]
            obs = {k: v for k, v in f.items() if k.startswith("observation.")}
            yield obs | {"task": f["task"]}, f["action"]
        return
    e = dict(np.load(ep))
    n = len(e["state"])
    action = e["state"][np.minimum(np.arange(n) + K, n - 1)].copy()
    action[:, 4] = hand_target(e["state"][:, 4], K)  # same labels as build_dataset.py
    for i in range(n):
        if np.abs(e["state"][min(i + K, n - 1)] - e["state"][i]).max() < 1e-3:
            continue  # idle frame: build_dataset.py drops these, so the policy never trained on them
        # same layout the dataset returns: CHW float in [0, 1]
        imgs = {f"observation.images.{c}": torch.from_numpy(e[c][i]).permute(2, 0, 1).float() / 255
                # for c in ("top", "left", "right", "wrist")}
                for c in ("top", "wrist")}
        yield imgs | {"observation.state": torch.from_numpy(e["state"][i]),
                      "task": "put the box on the car"}, torch.from_numpy(action[i])
cfg = PreTrainedConfig.from_pretrained(ckpt)
cfg.pretrained_path = ckpt
cfg.n_action_steps = 1  # re-plan every frame, so every prediction sees the current frame
rename = RENAME if cfg.type == "smolvla" else {}  # ACT was trained on the dataset's own camera names
policy = make_policy(cfg, ds_meta=ds.meta, rename_map=rename).eval()
pre, post = make_pre_post_processors(cfg, pretrained_path=ckpt, preprocessor_overrides={
    "device_processor": {"device": str(cfg.device)},
    "rename_observations_processor": {"rename_map": rename}})

pred, true, state = [], [], []
policy.reset()
with torch.no_grad():
    for obs, action in frames():
        if BLACK:
            obs = {k: v * 0 if k.startswith("observation.images") else v for k, v in obs.items()}
        state.append(obs["observation.state"].numpy())
        pred.append(post(policy.select_action(pre(obs))).squeeze(0).cpu().numpy())
        true.append(action.numpy())

pred, true, state = np.array(pred), np.array(true), np.array(state)
err = np.abs(pred - true)
hold = np.abs(state - true)  # baseline: "stay where you are"; a useful policy must beat it
names = ds.meta.features["action"]["names"]
print(f"episode {ep}, {len(true)} frames")
for j, n in enumerate(names):
    print(f"{n:24s} mean err {err[:, j].mean():.4f}  (hold {hold[:, j].mean():.4f})  "
          f"max {err[:, j].max():.4f}   range {np.ptp(true[:, j]):.3f}")
print(f"model / hold error: {err.mean() / hold.mean():.2f}   (< 1 means it predicts motion)")
np.savez(f"eval_{Path(ep).parent.name if npz else 'ep' + ep}.npz", pred=pred, true=true, state=state)
