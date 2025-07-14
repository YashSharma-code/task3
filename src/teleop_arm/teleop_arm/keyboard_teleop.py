import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty
import time
from rclpy.duration import Duration

class TeleopArm(Node):
    def __init__(self):
        super().__init__('teleop_arm_node')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.current_positions = [4.0,-1.5,0.0,0.0,0.0,0.0]

        self.key_bindings = {
            'q': (0, 0.1), 'a': (0, -0.1),
            'w': (1, 0.1), 's': (1, -0.1),
            'e': (2, 0.1), 'd': (2, -0.1),
            'r': (3, 0.1), 'f': (3, -0.1),
            't': (4, 0.1), 'g': (4, -0.1),
            'y': (5, 0.1), 'h': (5, -0.1)
        }

        print("🟢 Use Q/A, W/S, E/D, R/F, T/G, Y/H to control joints")
        print("🛑 Press and hold Ctrl+C to exit cleanly")

    def process_key(self, key):
        if key in self.key_bindings:
            joint_index, delta = self.key_bindings[key]
            self.current_positions[joint_index] += delta
            self.send_trajectory()

    def send_trajectory(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(seconds=0.5).to_msg()

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopArm()

    try:
        while rclpy.ok():
            key = node.get_key()
            node.process_key(key)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[EXIT] Teleoperation terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
