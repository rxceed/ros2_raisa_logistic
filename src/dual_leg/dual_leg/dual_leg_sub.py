import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import numpy as np

left_angle = 0
right_angle = 0

class subscriber(Node):
    def __init__(self):
        super().__init__("dual_leg_subscriber")
        self.sub_l = self.create_subscription(Float64, "dual_leg_l", self.callback_l, 10)
        self.sub_r = self.create_subscription(Float64, "dual_leg_r", self.callback_r, 10)
        self.sub_l
        self.sub_r
    def callback_l(self, msg):
        global left_angle
        self.get_logger().info(f"Heard L: {msg.data}")
        left_angle = msg.data
    def callback_r(self, msg):
        global right_angle
        self.get_logger().info(f"Heard R: {msg.data}")
        right_angle = msg.data

def main(arg=None):
    rclpy.init(args=arg)
    sub = subscriber()
    rclpy.spin(sub)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    