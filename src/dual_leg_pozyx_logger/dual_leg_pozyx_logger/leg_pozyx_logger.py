import csv
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from custom_interface.msg import DualLeg
import time
import os
import numpy as np

FILENAME = f"data_log_{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}.csv"

CURRENT_DIR = os.path.dirname("./leg_pozyx_log/")
CSV_PATH = os.path.join(CURRENT_DIR, FILENAME)

left_angle = 0
right_angle = 0
x_current = 0
y_current = 0
theta = 0
x_before = 0
y_before = 0
x_rel = 0
y_rel = 0

def log_to_csv(filename, data):
    """
    Appends a list of data as a row to a CSV file.
    """
    headers = ['timestamp', 'left_leg_cos', 'right_leg_cos', 'x_absolute', 'y_absolute', 'x_relative', 'y_relative', 'action']
    file_exists = os.path.isfile(filename)

    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        # Write header only if the file is new
        if not file_exists:
            writer.writerow(headers)

        writer.writerow(data)
        print(f"File location: {filename}")

class logger(Node):
    def __init__(self):
        super().__init__("leg_pozyx_logger")
        timer_period = 0.3
        self.sub_leg = self.create_subscription(DualLeg, "dual_leg", self.callback_leg, 10)
        self.sub_pozyx = self.create_subscription(Pose2D, "uwb_pose2d", self.callback_pozyx, 10)
        self.logger = self.create_timer(timer_period, self.callback_log)
    def callback_leg(self, msg):
        global right_angle, left_angle
        left_angle = msg.left_angle
        right_angle = msg.right_angle
    def callback_pozyx(self, msg):
        global x_current, y_current, theta
        x_current = msg.x
        y_current = msg.y
        theta = msg.theta
    def callback_log(self):
        global x_before, y_before, x_current, y_current, x_rel, y_rel, theta, left_angle, right_angle
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cos_r = np.cos(right_angle)
        cos_l = np.cos(left_angle)
        x_rel = abs(x_current - x_before)
        y_rel = abs(y_current - y_before)
        if cos_r <= 0.5 and cos_l <= 0.5:
            action = 0
        elif (x_rel >= 0.2 or y_rel >= 0.2) and (cos_r >= 0.8 and cos_l >= 0.8):
            action = 2
        elif (x_rel <= 0.2 or y_rel <= 0.2) and (cos_r >= 0.8 and cos_l >= 0.8):
            action = 1
        data = [current_time, cos_l, cos_r, x_current, y_current, x_rel, y_rel, action]
        x_before = x_current
        y_before = y_current
        log_to_csv(CSV_PATH, data)
        print(f"Logged: {data}")

def main(arg=None):
    rclpy.init(args=arg)
    sub = logger()
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