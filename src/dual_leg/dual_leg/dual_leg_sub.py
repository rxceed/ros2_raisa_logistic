import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from custom_interface.msg import DualLeg
import time
import numpy as np

left_angle = 0
right_angle = 0

class subscriber(Node):
    def __init__(self):
        super().__init__("dual_leg_subscriber")
        self.sub_leg = self.create_subscription(DualLeg, "dual_leg", self.callback_leg, 10)
        self.sub_leg
    def callback_leg(self, msg):
        global right_angle, left_angle
        right_angle = msg.right_angle
        left_angle = msg.left_angle
        cos_l = np.cos(left_angle)
        cos_r = np.cos(right_angle)
        self.get_logger().info(f"Heard Cos R: {cos_r} | Cos L: {cos_l}")

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
    