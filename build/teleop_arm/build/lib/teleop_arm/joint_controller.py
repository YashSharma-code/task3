#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_positions = [0.0] * 6
        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.publisher = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

    def callback(self, msg):
        self.joint_positions[0] += msg.linear.x
        self.joint_positions[1] += msg.linear.y
        self.joint_positions[2] += msg.linear.z

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.publisher.publish(traj)

def main():
    rclpy.init()
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()