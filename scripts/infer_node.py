"""Send camera images and joint states to learning/policy_server.py and publish its actions at 10 Hz.
python3 scripts/infer_node.py
"""
import numpy as np
import rclpy
import zmq
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
HAND = ["crane_plus_joint_hand"]
CAMS = {"/camera_top/image_raw": "top", "/camera_left/image_raw": "left",
        "/camera_right/image_raw": "right"}
SERVER = "tcp://127.0.0.1:5555"
STEP = 0.1  # 10 Hz, same as training


class PolicyClient(Node):
    def __init__(self):
        super().__init__("policy_client")
        self.sock = zmq.Context().socket(zmq.REQ)
        self.sock.setsockopt(zmq.RCVTIMEO, 5000)
        self.sock.connect(SERVER)
        self.first = True

        self.state, self.images = None, {}
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        for topic, name in CAMS.items():
            self.create_subscription(Image, topic, lambda m, n=name: self.on_image(n, m), 10)
        self.arm_pub = self.create_publisher(JointTrajectory, "/crane_plus_arm_controller/joint_trajectory", 10)
        self.hand_pub = self.create_publisher(JointTrajectory, "/crane_plus_gripper_controller/joint_trajectory", 10)
        self.create_timer(STEP, self.tick)

    def on_joints(self, m):
        pos = dict(zip(m.name, m.position))
        if all(j in pos for j in ARM + HAND):
            self.state = np.array([pos[j] for j in ARM + HAND], np.float32)

    def on_image(self, name, m):
        # same decoding as convert2npz.py: HWC uint8, first 3 channels
        self.images[name] = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[..., :3].copy()

    def tick(self):
        if self.state is None or len(self.images) < len(CAMS):
            return  # waiting for the first messages
        self.sock.send_pyobj({**self.images, "state": self.state, "reset": self.first})
        self.first = False
        try:
            action = self.sock.recv_pyobj()
        except zmq.Again:
            self.get_logger().error("policy server did not answer within 5 s")
            raise SystemExit(1)
        self.publish(self.arm_pub, ARM, action[:4])
        self.publish(self.hand_pub, HAND, action[4:])

    def publish(self, pub, names, positions):
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions],
                                  time_from_start=Duration(sec=0, nanosec=int(STEP * 1e9)))
        pub.publish(JointTrajectory(joint_names=names, points=[pt]))


def main():
    rclpy.init()
    rclpy.spin(PolicyClient())


if __name__ == "__main__":
    main()
