import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class TwistTeleop(Node):
    def __init__(self):
        super().__init__('twist_teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        print("🟢 Twist Teleop Node Started")
        print("Use keys: W/S (Z+/-), A/D (Y+/-), Q/E (X+/-)")
        print("Ctrl+C to exit")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        try:
            while True:
                key = self.get_key()
                twist = Twist()
                if key == 'w':
                    twist.linear.z = 0.1
                elif key == 's':
                    twist.linear.z = -0.1
                elif key == 'a':
                    twist.linear.y = 0.1
                elif key == 'd':
                    twist.linear.y = -0.1
                elif key == 'q':
                    twist.linear.x = 0.1
                elif key == 'e':
                    twist.linear.x = -0.1
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    continue
                self.publisher_.publish(twist)
        except KeyboardInterrupt:
            pass
        finally:
            print("\n🛑 Exiting Twist Teleop")

def main(args=None):
    rclpy.init(args=args)
    node = TwistTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

