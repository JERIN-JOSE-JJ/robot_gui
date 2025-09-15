import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool

class ROSPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.cmd_vel_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        self.servo_pub = self.create_publisher(String, '/servo_control', 10)
        self.auto_enable_pub = self.create_publisher(Bool, '/autonomy_enable', 10)

    def publish_cmd(self, direction):
        msg = Twist()
        if direction == "forward": msg.linear.x = 0.5
        elif direction == "backward": msg.linear.x = -0.5
        elif direction == "left": msg.angular.z = 0.5
        elif direction == "right": msg.angular.z = -0.5
        elif direction == "stop": msg.linear.x = msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def publish_mode(self, mode_str):
        msg = String()
        msg.data = mode_str
        self.mode_pub.publish(msg)

    def publish_servo(self, joint, angle):
        msg = String()
        msg.data = f"{joint}:{angle}"
        self.servo_pub.publish(msg)

    def enable_autonomy(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.auto_enable_pub.publish(msg)
