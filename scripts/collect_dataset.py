"""Record one expert demonstration per scene in datasets/."""
import json
import os
import signal
import subprocess
import time
import shutil
from pathlib import Path
from sim_utils import DATASET_DIR, load_scene, place_scene, read_pose
from convert2npz import export

# No /clock: ~1 kHz of tiny messages overflows the writer; message stamps suffice.
TOPICS = [
    "/joint_states",
    "/camera_top/image_raw", "/camera_left/image_raw", "/camera_right/image_raw"
]
# Record on the container's own disk; the bind-mounted repo is too slow for raw images.
LOCAL = Path.home() / "bags_temp"
XY_TOL, Z_TOL = 0.03, 0.01  # m; box center vs roof center, box bottom vs roof top

def box_on_roof(ep_dir, cfg) -> dict:
    """Read the final box pose from Gazebo and check it rests on the car roof."""
    roof = json.loads((ep_dir / "initial_pose.json").read_text(encoding="utf-8"))["scene"]["car"]
    time.sleep(1.0)  # let the dropped box settle
    x, y, z = read_pose(cfg["box"]["name"])
    roof_z = cfg["box"]["spawn_z"] + cfg["car"]["roof_z"]  # Gazebo z of the roof top
    xy_err = ((x - roof["roof_x"]) ** 2 + (y - roof["roof_y"]) ** 2) ** 0.5
    return {"success": xy_err < XY_TOL and abs(z - roof_z) < Z_TOL,
            "final_box": {"x": x, "y": y, "z": z}, "xy_err": xy_err}

def record_episode(ep_dir, cfg) -> int:
    s = json.loads((ep_dir / "initial_pose.json").read_text(encoding="utf-8"))["scene"]
    name = ep_dir.relative_to(DATASET_DIR).as_posix()  # e.g. "train/random_..._000013"
    bag_tmp = LOCAL / ep_dir.name
    shutil.rmtree(bag_tmp, ignore_errors=True)
    LOCAL.mkdir(parents=True, exist_ok=True)
    place_scene("default", s, cfg)
    time.sleep(1.0)

    bag = subprocess.Popen(["ros2", "bag", "record", "-s", "mcap", "--max-cache-size", "1000000000",
                            "-o", str(bag_tmp), *TOPICS])
    try:
        time.sleep(2.0)
        expert = subprocess.run(
            ["ros2", "launch", "crane_dataset_expert", "dataset_expert.launch.py"],
            env={**os.environ, "EPISODE": name},  # the launch file reads datasets/<EPISODE>/initial_pose.json
        )
    finally:
        bag.send_signal(signal.SIGINT)
        bag.wait(timeout=30)
    shutil.move(str(bag_tmp), str(ep_dir / "bag"))
    return expert.returncode

def main():
    published = subprocess.run(["ros2", "topic", "list"],
                               capture_output=True, text=True).stdout.split()
    missing = [t for t in TOPICS if t not in published]
    if missing:
        raise SystemExit(f"Topics not published (is the camera bridge running?): {missing}")

    cfg = load_scene()

    # iterate on all episodes in the dataset
    for ep_dir in sorted(p.parent for p in DATASET_DIR.rglob("initial_pose.json")):
        result = ep_dir / "result.json"
        if result.exists():
            continue
        # a bag without result.json is from an interrupted run: start the episode over
        shutil.rmtree(ep_dir / "bag", ignore_errors=True)

        print(f"== {ep_dir.name} ==")
        code = record_episode(ep_dir, cfg)
        outcome = {"expert_exit": code, **box_on_roof(ep_dir, cfg)}
        result.write_text(json.dumps(outcome))
        if code == 0 and outcome["success"]:  # same filter as convert2npz.py
            export(ep_dir)

if __name__ == "__main__":
    main()
