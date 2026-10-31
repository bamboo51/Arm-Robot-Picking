"""Turn datasets/<ep>/bag into datasets/<ep>/episode.npz at a fixed rate"""
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState

FPS = 10
JOINTS = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4", "crane_plus_joint_hand"]
CAMS = {"/camera_top/image_raw": "top",
        # "/camera_left/image_raw": "left", "/camera_right/image_raw": "right",
        "/camera_wrist/image_raw": "wrist"}

def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    joints, frames = [], {name: [] for name in CAMS.values()}

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/joint_states":
            m = deserialize_message(data, JointState)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            pos = dict(zip(m.name, m.position))
            joints.append((t, [pos[j] for j in JOINTS]))
        elif topic in CAMS:
            m = deserialize_message(data, Image)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[..., :3]
            frames[CAMS[topic]].append((t, img))
    return joints, frames

def nearest(stamped, t):
    """Latest sample at or before t (zero-order hold): at run time the policy never sees a future message."""
    times = np.array([s for s, _ in stamped])
    return stamped[max(int(np.searchsorted(times, t, side="right")) - 1, 0)][1]

def export(ep_dir):
    joints, frames = read_bag(ep_dir / "bag")
    # bag order is arrival order; nearest() needs each stream sorted by header stamp
    joints = sorted(joints, key=lambda s: s[0])
    frames = {k: sorted(v, key=lambda s: s[0]) for k, v in frames.items()}
    start = max(joints[0][0], *(f[0][0] for f in frames.values()))
    end = min(joints[-1][0], *(f[-1][0] for f in frames.values()))
    ticks = np.arange(start, end, 1.0/FPS)

    state = np.array([nearest(joints, t) for t in ticks], np.float32)
    action = np.vstack([state[1:], state[-1:]])
    images = {k: np.stack([nearest(v, t) for t in ticks]) for k, v in frames.items()}
    np.savez_compressed(ep_dir / "episode.npz", state=state, action=action, **images)

if __name__ == "__main__":
    import json
    from sim_utils import DATASET_DIR

    for result in sorted(DATASET_DIR.rglob("result.json")):  # recorded episodes in train/, val/, test/
        ep_dir = result.parent
        # episodes recorded before the roof check have no "success" key; keep them
        r = json.loads(result.read_text())
        if r.get("expert_exit") != 0 or not r.get("success", True):
            continue
        if (ep_dir / "episode.npz").exists():
            continue

        print(f"== {ep_dir.relative_to(DATASET_DIR)} ==")
        export(ep_dir)
