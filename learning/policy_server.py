"""
Serve a trained LeRobot policy over ZMQ for scripts/infer_node.py
/opt/lerobot-venv/bin/python learning/policy_server.py <checkpoint>/pretrained_model
"""
import os
import sys
from pathlib import Path

import torch
import zmq
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.policies import make_policy, make_pre_post_processors

REPO = Path(__file__).resolve().parents[1]
RENAME = {"observation.images.top": "observation.images.camera1",
          "observation.images.left": "observation.images.camera2",
          "observation.images.right": "observation.images.camera3"}

ckpt = sys.argv[1]
cfg = PreTrainedConfig.from_pretrained(ckpt)
cfg.pretrained_path = ckpt  # without this make_policy builds random weights
# N=1 re-plans every tick; from rest the first action is ~"stay", so N>1 executes more of the chunk to get moving
cfg.n_action_steps = int(os.environ.get("N", 1))
rename = RENAME if cfg.type == "smolvla" else {}
meta = LeRobotDatasetMetadata("local/crane_pick_place", root=REPO/"lerobot_data/crane_pick_place")
policy = make_policy(cfg, ds_meta=meta, rename_map=rename).eval()
pre, post = make_pre_post_processors(cfg, pretrained_path=ckpt, preprocessor_overrides={
    "device_processor": {"device": str(cfg.device)},
    "rename_observations_processor": {"rename_map": rename}
})

sock = zmq.Context().socket(zmq.REP)
sock.bind("tcp://127.0.0.1:5555") # same container
print("policy server ready on :5555", flush=True)

while True:
    msg = sock.recv_pyobj()
    if msg.get("reset"):
        policy.reset()
    # obs = {f"observation.images.{c}": torch.from_numpy(msg[c]).permute(2, 0, 1).float() / 255 for c in ("top", "left", "right", "wrist")}
    obs = {f"observation.images.{c}": torch.from_numpy(msg[c]).permute(2, 0, 1).float() / 255 for c in ("top", "wrist")}
    obs["observation.state"] = torch.from_numpy(msg["state"])
    obs["task"] = msg.get("task", "put the box on the car")
    with torch.no_grad():
        action = post(policy.select_action(pre(obs))).squeeze(0).cpu().numpy()
    sock.send_pyobj(action.tolist())
