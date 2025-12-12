import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import requests
import threading
import time
import numpy as np
from dual_leg.secret import post_nav_url, get_pose_url

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

def post_nav(nav_payload):
    res = requests.post(post_nav_url, json=nav_payload, headers={'Content-Type': 'application/json'})
    return res

def get_pose():
    res = requests.get(get_pose_url)
    return res.json()

def nav():
    global left_angle
    while True:
        if left_angle > 30.0:
            x = np.float64(get_pose()['x'])
            y = np.float64(get_pose()['y'])
            theta = np.float64(get_pose()['theta'])
            payload = {'x': x+1, 'y': y, 'theta': theta}
            res = post_nav(payload)
            print(res.json())

def main(arg=None):
    display_thread = threading.Thread(target=nav, daemon=True)
    display_thread.start()

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
    