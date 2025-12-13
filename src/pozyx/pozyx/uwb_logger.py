import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import time

class subscriber(Node):
    def __init__(self):
        super().__init__('uwb_pose2d_subscriber')
        self.sub = self.create_subscription(Pose2D, 'uwb_pose2d', self.callback, 10)
        self.sub
    def callback(self, msg: Pose2D):
        x = msg.x
        y = msg.y
        theta = msg.theta
        self.get_logger().info(f'X: {x} | Y: {y} | theta: {theta}')

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