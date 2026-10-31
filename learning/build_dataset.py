from lerobot.datasets.lerobot_dataset import LeRobotDataset
from pathlib import Path
import os
import numpy as np
from gripper_label import hand_target

REPO = Path(__file__).resolve().parents[1]
K = int(os.environ.get("K", 5))  # action = joint angles K frames (K/10 s) ahead; eval_offline.py needs the same K
JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3",
          "crane_plus_joint4", "crane_plus_joint_hand"] 
features = {
    "observation.state":    {"dtype": "float32", "shape": (5,), "names": JOINTS},
    "action":               {"dtype": "float32", "shape": (5,), "names": JOINTS},
    **{f"observation.images.{c}": {"dtype": "video", "shape": (240, 320, 3), "names": ["height", "width", "channels"]}
       # for c in ("top", "left", "right", "wrist")
       for c in ("top", "wrist")
       }
}
ds = LeRobotDataset.create(
    repo_id="local/crane_pick_place", fps=10, 
    root=REPO /"lerobot_data/crane_pick_place", 
    features=features)

for npz in sorted((REPO / "datasets" / "train").glob("*/episode.npz")):  # val/ and test/ stay out of training
    e = dict(np.load(npz))  # NpzFile re-decompresses the whole array on every e[key]
    n = len(e["state"])
    action = e["state"][np.minimum(np.arange(n) + K, n - 1)].copy()
    action[:, 4] = hand_target(e["state"][:, 4], K)
    for i in range(n):
        # skip idle frames (nothing moves in the next K frames): their "when to start" is invisible, so the
        # policy learns to stay put and never leaves the start pose
        if np.abs(e["state"][min(i + K, n - 1)] - e["state"][i]).max() < 1e-3:
            continue
        ds.add_frame({
            "observation.state": e["state"][i],
            "action": action[i],
            "observation.images.top": e["top"][i],
            # "observation.images.left": e["left"][i],
            # "observation.images.right": e["right"][i],
            "observation.images.wrist": e["wrist"][i],
            "task": "put the box on the car",
        })
    ds.save_episode()
ds.finalize()  # flushes the last metadata batch (buffered 10 episodes at a time)
