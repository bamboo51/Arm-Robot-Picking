"""Gripper action = what the expert commanded, not where the fingers stopped.

The expert commands close = +10 deg (dataset_expert.cpp GRIPPER_CLOSE), but /joint_states stops at the box,
so the measured angle teaches the policy to close only to the contact angle and the box slips.
python gripper_label.py  runs the self-check.
"""
import numpy as np

CLOSE = np.radians(10)  # dataset_expert.cpp GRIPPER_CLOSE
OPENED = np.radians(-20)  # below this the gripper is open (GRIPPER_OPEN = -30 deg)
MOVE = 0.01  # rad per frame; smaller changes are contact jitter


def hand_target(hand: np.ndarray, k: int) -> np.ndarray:
    """Gripper target k frames ahead: the measured angle, except CLOSE from the start of the grasp until the release."""
    n = len(hand)
    ahead = np.minimum(np.arange(n) + k, n - 1)
    target = hand[ahead].copy()
    d = np.diff(hand)
    # frames where a close starts from open; an episode can start open (left by the previous one), and then the
    # expert's "initialize gripper" (to 0) is also a close from open, so the grasp is the LAST one
    rising = (hand[:-1] < OPENED) & (d > MOVE)
    edges = np.flatnonzero(rising & ~np.r_[False, rising[:-1]])
    if not len(edges):
        return target  # no close from open, so no grasp to relabel
    start = edges[-1] + 1
    reopening = np.flatnonzero(d[start:] < -MOVE)
    end = start + reopening[0] + 1 if len(reopening) else n
    target[(ahead >= start) & (ahead < end)] = CLOSE
    return target


if __name__ == "__main__":
    # default 0 -> open -0.52 -> close, stopped by the box at -0.2 -> hold -> release to -0.52
    hand = np.array([0, 0, -0.3, -0.52, -0.52, -0.4, -0.25, -0.2, -0.2, -0.2, -0.35, -0.52, -0.52])
    t = hand_target(hand, 0)
    assert np.allclose(t[:5], hand[:5]), t  # before the grasp: unchanged
    assert np.allclose(t[5:10], CLOSE), t  # closing and holding: commanded close
    assert np.allclose(t[10:], hand[10:]), t  # release: unchanged
    assert np.allclose(hand_target(hand, 2)[3:8], CLOSE)  # k ahead shifts the window earlier
    # episode that starts open: initialize (to 0) -> open -> grasp at +0.09 -> release (recorded episode 000003)
    hand = np.array([-0.52, -0.52, -0.3, 0, 0, -0.3, -0.52, -0.52, -0.2, 0.09, 0.09, -0.3, -0.52])
    t = hand_target(hand, 0)
    assert np.allclose(t[:8], hand[:8]), t  # the initialize close is not the grasp
    assert np.allclose(t[8:11], CLOSE), t  # the grasp is
    assert np.allclose(t[11:], hand[11:]), t
    print("gripper_label self-check passed")
