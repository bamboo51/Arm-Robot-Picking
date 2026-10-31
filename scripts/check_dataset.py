"""Check every recorded episode before splitting into train/val/test. Prints one line per episode and flags bad ones.

python3 scripts/check_dataset.py            # all episodes under datasets/
python3 scripts/check_dataset.py --move     # also move flagged episodes to datasets/rejected/
"""
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "learning"))
from gripper_label import CLOSE, hand_target  # noqa: E402
from sim_utils import DATASET_DIR  # noqa: E402

K = 5  # same as learning/build_dataset.py
MAX_STEP = 0.15  # rad per frame (10 Hz); more means the motion is too fast to learn from
MAX_FROZEN = 0.10  # share of moving frames whose image did not change; more means dropped camera frames
MIN_FRAMES = 50  # 5 s; shorter means the expert stopped early


def check(ep: Path) -> list[str]:
    """Problems with one episode; empty list means it is fine."""
    result = json.loads((ep / "result.json").read_text())
    if result.get("expert_exit") != 0 or not result.get("success"):
        return [f"expert failed (exit {result.get('expert_exit')}, success {result.get('success')})"]
    if not (ep / "episode.npz").exists():
        return ["no episode.npz (run convert2npz.py)"]

    e = np.load(ep / "episode.npz")
    s = e["state"]
    problems = []
    if len(s) < MIN_FRAMES:
        problems.append(f"only {len(s)} frames")
    step = np.abs(np.diff(s, axis=0)).max()
    if step > MAX_STEP:
        problems.append(f"joint jumps {step:.2f} rad in one frame")

    moving = np.abs(np.diff(s, axis=0)).max(1) > 1e-3
    for cam in (k for k in e.files if k not in ("state", "action")):
        im = e[cam]
        if im.mean() < 5:
            problems.append(f"{cam} is black")
            continue
        frozen = np.array([np.array_equal(im[i], im[i - 1]) for i in range(1, len(im))])
        if moving.any() and frozen[moving].mean() > MAX_FROZEN:
            problems.append(f"{cam} frozen in {frozen[moving].mean():.0%} of moving frames")

    closed = np.isclose(hand_target(s[:, 4], 0), CLOSE, atol=1e-4)  # state is float32, CLOSE float64: == never matches
    if not closed.any():
        problems.append("no grasp found in the gripper angle")
    elif closed.sum() < 10:
        problems.append(f"grasp held only {closed.sum()} frames")
    return problems


def main():
    episodes = sorted(p.parent for p in DATASET_DIR.rglob("result.json") if "rejected" not in p.parts)
    bad = {}
    for ep in episodes:
        problems = check(ep)
        print(f"{'BAD ' if problems else 'ok  '} {ep.relative_to(DATASET_DIR)}  {'; '.join(problems)}")
        if problems:
            bad[ep] = problems
    print(f"\n{len(episodes) - len(bad)}/{len(episodes)} episodes ok")

    if "--move" in sys.argv and bad:
        rejected = DATASET_DIR / "rejected"
        rejected.mkdir(exist_ok=True)
        for ep in bad:
            shutil.move(str(ep), str(rejected / ep.name))
        print(f"moved {len(bad)} episodes to {rejected}")


if __name__ == "__main__":
    main()
