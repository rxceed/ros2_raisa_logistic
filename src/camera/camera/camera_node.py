import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Viewer(Node):
    def __init__(self):
        super().__init__('viewer')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, 'camera/image_raw', self.callback, 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Viewer", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Viewer()
    rclpy.spin(node)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
