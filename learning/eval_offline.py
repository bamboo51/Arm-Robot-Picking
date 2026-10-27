"""Replay one recorded episode through a trained policy and compare its actions to the expert's.

uv run eval_offline.py <checkpoint>/pretrained_model [episode_index | path/to/episode.npz]
An episode.npz lets you test an episode that is not in the training set.
"""
import sys
from pathlib import Path

import numpy as np
import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies import make_policy, make_pre_post_processors

ROOT = Path.home() / "arm/lerobot_data/crane_pick_place"
RENAME = {"observation.images.top": "observation.images.camera1",
          "observation.images.left": "observation.images.camera2",
          "observation.images.right": "observation.images.camera3"}

if len(sys.argv) < 2:
    sys.exit(__doc__)
ckpt, ep = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "0"
npz = ep.endswith(".npz")
ds = LeRobotDataset("local/crane_pick_place", root=ROOT, episodes=[0 if npz else int(ep)])

def frames():
    """Yield (observation, expert action) per frame, from the dataset or from a raw episode.npz."""
    if not npz:
        for i in range(len(ds)):
            f = ds[i]
            obs = {k: v for k, v in f.items() if k.startswith("observation.")}
            yield obs | {"task": f["task"]}, f["action"]
        return
    e = dict(np.load(ep))
    for i in range(len(e["state"])):
        # same layout the dataset returns: CHW float in [0, 1]
        imgs = {f"observation.images.{c}": torch.from_numpy(e[c][i]).permute(2, 0, 1).float() / 255
                for c in ("top", "left", "right")}
        yield imgs | {"observation.state": torch.from_numpy(e["state"][i]),
                      "task": "put the box on the car"}, torch.from_numpy(e["action"][i])
cfg = PreTrainedConfig.from_pretrained(ckpt)
cfg.pretrained_path = ckpt
rename = RENAME if cfg.type == "smolvla" else {}  # ACT was trained on the dataset's own camera names
policy = make_policy(cfg, ds_meta=ds.meta, rename_map=rename).eval()
pre, post = make_pre_post_processors(cfg, pretrained_path=ckpt, preprocessor_overrides={
    "device_processor": {"device": str(cfg.device)},
    "rename_observations_processor": {"rename_map": rename}})

pred, true = [], []
policy.reset()
with torch.no_grad():
    for obs, action in frames():
        pred.append(post(policy.select_action(pre(obs))).squeeze(0).cpu().numpy())
        true.append(action.numpy())

pred, true = np.array(pred), np.array(true)
err = np.abs(pred - true)
names = ds.meta.features["action"]["names"]
print(f"episode {ep}, {len(true)} frames")
for j, n in enumerate(names):
    print(f"{n:24s} mean err {err[:, j].mean():.4f} rad   max {err[:, j].max():.4f}   range {np.ptp(true[:, j]):.3f}")
np.savez(f"eval_{Path(ep).parent.name if npz else 'ep' + ep}.npz", pred=pred, true=true)
