"""
Script for calculate the reachability of the crane.
This will be used for range definition of the crane.

Script for get Unified Robot Description Format (URDF) of the crane.
```bash
source ~/airobot_ws/install/setup.bash
xacro $(ros2 pkg prefix crane_plus_description)/share/crane_plus_description/urdf/crane_plus.urdf.xacro > crane_plus.urdf
```

Joint 1: base (rotates the whole arm around the vertical axis)
"""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = REPO_ROOT / "models" / "crane_plus" / "crane_plus.urdf"
SCENE_FILE = REPO_ROOT / "config" / "scene.yaml"


def read_urdf(file: Path) -> ET.Element:
    return ET.parse(file).getroot()


def find_joint(root: ET.Element, name: str) -> ET.Element:
    joint = root.find(f"joint[@name='{name}']")
    if joint is None:
        raise KeyError(f"joint {name} not found in URDF")
    return joint

def safe_acos(value: float) -> float:
    # Rounding can push the argument just outside [-1, 1] at the workspace edge.
    return math.acos(max(-1.0, min(1.0, value)))


def ik(r: float, z: float, L1: float, L2: float, h: float) -> dict:
    """
    Planar IK for the wrist at (r, z) with the gripper pointing straight down.

    Angles are measured from vertical, positive = tilting forward.
    URDF signs (checked against TF): q2 = a2, q3 = -b3, q4 = -b4.
    Returns {} when the point is out of reach.
    """
    # shoulder to wrist
    dz = z - h
    d = math.hypot(r, dz)

    if d > L1 + L2 or d < abs(L1 - L2):
        return {}

    # shoulder wrist line angle
    alpha = math.atan2(r, dz)
    # angle at the shoulder
    beta = safe_acos((L1**2 + d**2 - L2**2) / (2 * L1 * d))
    # angle at the elbow
    gamma = safe_acos((L1**2 + L2**2 - d**2) / (2 * L1 * L2))
    return {
        "elbow_up": {
            "q2": alpha - beta,
            "q3": -(math.pi - gamma),
            "q4": alpha - beta - gamma,
        },
        "elbow_down": {
            "q2": alpha + beta,
            "q3": math.pi - gamma,
            "q4": alpha + beta + gamma - 2 * math.pi,
        },
    }


def fk(q2: float, q3: float, q4: float, L1: float, L2: float, h: float):
    """Wrist (r, z) and gripper angle from vertical, for URDF joint values."""
    a2 = q2
    a3 = a2 - q3
    a4 = a3 - q4
    r = L1 * math.sin(a2) + L2 * math.sin(a3)
    z = h + L1 * math.cos(a2) + L2 * math.cos(a3)
    return r, z, a4

def joint_limits(root: ET.Element, name: str) -> tuple[float, float]:
    limit = find_joint(root, name).find("limit")
    if limit is None:
        raise KeyError(f"joint {name} has no <limit> in URDF")
    return float(limit.attrib["lower"]), float(limit.attrib["upper"])


def joint_origin_z(root: ET.Element, name: str) -> float:
    # A missing <origin> means 0 0 0 in URDF.
    origin = find_joint(root, name).find("origin")
    if origin is None:
        return 0.0
    return float(origin.get("xyz", "0 0 0").split()[2])

def reachable(r: float, z: float, L1: float, L2: float, h: float,
              limits: dict[str, tuple[float, float]]) -> bool:
    """True if any elbow branch keeps q2, q3, q4 inside their URDF limits."""
    for q in ik(r, z, L1, L2, h).values():
        if all(limits[name][0] <= value <= limits[name][1] for name, value in q.items()):
            return True
    return False

def reachable_grid(L1, L2, h, limits, table_z: float) -> np.ndarray:
    """All reachable (r, z) wrist points on a 2 mm grid, as an (N, 2) array.

    r spans the arm's full reach; z spans the table top up to the arm pointing straight up.
    """
    rs = np.arange(0, L1 + L2, 0.002)
    zs = np.arange(table_z, h + L1 + L2, 0.002)

    reachable_points = []
    for r in rs:
        for z in zs:
            if reachable(r, z, L1, L2, h, limits):
                reachable_points.append((r, z))

    return np.array(reachable_points)

def valid_r_range(z_low: float, z_high: float, L1, L2, h, limits) -> tuple[float, float] | None:
    """Smallest and largest wrist r where every height from z_low to z_high is reachable.

    The arm moves straight up and down over an object, so the whole column must work.
    Returns None if no r works.
    """
    zs = np.arange(z_low, z_high + 1e-9, 0.002)
    valid = [
        r for r in np.arange(0, L1 + L2, 0.001)
        if all(reachable(r, z, L1, L2, h, limits) for z in zs)
    ]
    if not valid:
        return None
    return float(min(valid)), float(max(valid))

def object_r_range(wrist_range: tuple[float, float], hand_offset: float, margin: float,
                   base_clearance: float) -> tuple[float, float] | None:
    """Object r range from the wrist r range, shrunk by safety margins. None if nothing is left."""
    r_min, r_max = wrist_range

    # shift both by the hand offset (the object is 1cm further than the wrist)
    r_min = r_min + hand_offset
    r_max = r_max + hand_offset

    # margin
    r_min = r_min + margin
    r_max = r_max - margin

    r_min = max(r_min, base_clearance)
    if r_min > r_max:
        return None
    return r_min, r_max

def self_check(L1, L2, h, limits):
    """Known poses measured in the sim. Fails loudly if the IK/FK math or signs break."""
    # Home pose: /joint_states vs TF (r -0.006, z 0.166, gripper ~90 deg).
    r, z, a4 = fk(-1.160, -2.010, -0.730, L1, L2, h)
    assert math.isclose(r, -0.006, abs_tol=0.001), f"fk home r={r:.4f}"
    assert math.isclose(z, 0.166, abs_tol=0.001), f"fk home z={z:.4f}"
    assert math.isclose(math.degrees(a4), 90.5, abs_tol=0.5), f"fk home angle={math.degrees(a4):.1f}"

    # Grasp pose: elbow-up joint values.
    q = ik(0.09, 0.14, L1, L2, h)["elbow_up"]
    for name, expected in [("q2", -0.029), ("q3", -1.754), ("q4", -1.417)]:
        assert math.isclose(q[name], expected, abs_tol=0.01), f"ik grasp {name}={q[name]:.3f}"

    # Round trip: fk(ik(p)) must return p with the gripper pointing down.
    for branch, q in ik(0.09, 0.14, L1, L2, h).items():
        r, z, a4 = fk(q["q2"], q["q3"], q["q4"], L1, L2, h)
        assert math.isclose(r, 0.09, abs_tol=1e-6) and math.isclose(z, 0.14, abs_tol=1e-6), \
            f"round trip {branch}: r={r:.4f} z={z:.4f}"
        assert math.isclose(math.degrees(a4), 180.0, abs_tol=0.01), \
            f"round trip {branch}: angle={math.degrees(a4):.2f}"

    # Reachability seen in the sim.
    assert reachable(0.09, 0.14, L1, L2, h, limits), "grasp should be reachable"
    assert reachable(0.09, 0.173, L1, L2, h, limits), "lift to 0.173 should be reachable"
    assert not reachable(0.09, 0.18, L1, L2, h, limits), "lift stopped below 0.18"
    assert not reachable(0.09, 0.30, L1, L2, h, limits), "0.30 is too high"
    print("self_check passed")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    args = parser.parse_args()

    if args.urdf.suffix != ".urdf":
        parser.error(f"Expected a .urdf file: {args.urdf}")
    if not args.urdf.is_file():
        parser.error(f"URDF not found: {args.urdf}")

    root = read_urdf(args.urdf)

    L1 = joint_origin_z(root, "crane_plus_joint3")
    L2 = joint_origin_z(root, "crane_plus_joint4")
    h = joint_origin_z(root, "crane_plus_joint1") + joint_origin_z(root, "crane_plus_joint2")

    limits = {
        "q2": joint_limits(root, "crane_plus_joint2"),
        "q3": joint_limits(root, "crane_plus_joint3"),
        "q4": joint_limits(root, "crane_plus_joint4"),
    }
    self_check(L1, L2, h, limits)

    # Table top in MoveIt's frame (origin = robot base).
    scene = yaml.safe_load(SCENE_FILE.read_text(encoding="utf-8"))
    table_z = scene["table"]["top_z"] - scene["robot"]["base_z"]

    # Wrist heights the expert uses (same formulas as dataset_expert.cpp).
    grasp = scene["grasp"]
    grasp_z = grasp["grasp_z"]
    release_z = scene["car"]["roof_z"] + (grasp_z - table_z) + grasp["release_clearance"]
    transfer_z = max(grasp["approach_z"], release_z + grasp["transfer_clearance"])
    print(f"wrist heights: grasp {grasp_z:.3f}, release {release_z:.3f}, transfer {transfer_z:.3f}")

    sampling = scene["sampling"]
    snippet = []
    for name, z_low in [("box", grasp_z), ("car", release_z)]:
        wrist_range = valid_r_range(z_low, transfer_z, L1, L2, h, limits)
        if wrist_range is None:
            print(f"{name}: no reachable r for wrist z {z_low:.3f} -> {transfer_z:.3f}")
            continue
        print(f"{name}: wrist r {wrist_range[0]:.3f} - {wrist_range[1]:.3f} "
              f"(z {z_low:.3f} -> {transfer_z:.3f})")

        r_range = object_r_range(wrist_range, grasp["hand_offset_y"],
                                 sampling["margin"], sampling["base_clearance"])
        if r_range is None:
            print(f"{name}: nothing left after margin and base clearance")
            continue
        snippet.append(f"  {name}_r: [{r_range[0]:.3f}, {r_range[1]:.3f}]")

    print("\nPaste into config/scene.yaml under sampling:")
    print("\n".join(snippet))

if __name__ == "__main__":
    main()
