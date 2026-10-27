from lerobot.datasets.lerobot_dataset import LeRobotDataset
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[1]
JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3",
          "crane_plus_joint4", "crane_plus_joint_hand"] 
features = {
    "observation.state":    {"dtype": "float32", "shape": (5,), "names": JOINTS},
    "action":               {"dtype": "float32", "shape": (5,), "names": JOINTS},
    **{f"observation.images.{c}": {"dtype": "video", "shape": (240, 320, 3), "names": ["height", "width", "channels"]}
       for c in ("top", "left", "right")
       }
}
ds = LeRobotDataset.create(
    repo_id="local/crane_pick_place", fps=10, 
    root=REPO /"lerobot_data/crane_pick_place", 
    features=features)

for npz in sorted((REPO / "datasets").glob("*/episode.npz")):
    e = dict(np.load(npz))  # NpzFile re-decompresses the whole array on every e[key]
    for i in range(len(e["state"])):
        ds.add_frame({
            "observation.state": e["state"][i],
            "action": e["action"][i],
            "observation.images.top": e["top"][i],
            "observation.images.left": e["left"][i],
            "observation.images.right": e["right"][i],
            "task": "put the box on the car",
        })
    ds.save_episode()
ds.finalize()  # flushes the last metadata batch (buffered 10 episodes at a time)
