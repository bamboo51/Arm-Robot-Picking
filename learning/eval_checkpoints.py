"""Plot how the policy's offline error changes during training: one eval_offline.py run per checkpoint.

uv run eval_checkpoints.py <output_dir> <train episode index> <test episode.npz>
e.g. uv run eval_checkpoints.py ~/arm/outputs/act_crane 0 ~/arm/datasets/test/<scene>/episode.npz
Writes training_curve.png: error on a training episode (did it learn?) and an unseen one (does it generalize?).
"""
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

if len(sys.argv) < 4:
    sys.exit(__doc__)
out, train_ep, test_ep = Path(sys.argv[1]).expanduser(), sys.argv[2], sys.argv[3]
here = Path(__file__).parent
ckpts = sorted(p for p in (out / "checkpoints").iterdir() if p.name.isdigit())  # skip the "last" link

curves = {"train": [], "test": []}
for c in ckpts:
    for split, ep in (("train", train_ep), ("test", test_ep)):
        print(f"== step {int(c.name)}  {split}", flush=True)
        subprocess.run([sys.executable, here / "eval_offline.py", c / "pretrained_model", ep], check=True)
        d = np.load(f"eval_{Path(ep).parent.name if ep.endswith('.npz') else 'ep' + ep}.npz")
        curves[split].append((int(c.name), np.abs(d["pred"] - d["true"]).mean(),
                              np.abs(d["state"] - d["true"]).mean(),
                              np.abs(d["pred"][:, 4] - d["true"][:, 4]).mean()))

fig, ax = plt.subplots(1, 2, figsize=(12, 4))
for split, rows in curves.items():
    step, err, hold, hand = np.array(rows).T
    ax[0].plot(step, err, "o-", label=f"{split}: policy")
    ax[0].plot(step, hold, ":", label=f"{split}: hold baseline")
    ax[1].plot(step, hand, "o-", label=split)
ax[0].set(title="mean joint error vs expert (rad)", xlabel="training step")
ax[1].set(title="gripper error (rad)", xlabel="training step")
for a in ax:
    a.legend()
plt.tight_layout()
plt.savefig("training_curve.png")
print("saved training_curve.png")
