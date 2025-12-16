import csv
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import os
import math

FILENAME = f"data_log_{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}.csv"

CURRENT_DIR = os.path.dirname("./dual_leg_logs/")
CSV_PATH = os.path.join(CURRENT_DIR, FILENAME)

left_angle = 0
right_angle = 0

def log_to_csv(filename, data):
    """
    Appends a list of data as a row to a CSV file.
    """
    headers = ['timestamp', 'left', 'right']
    file_exists = os.path.isfile(filename)

    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        # Write header only if the file is new
        if not file_exists:
            writer.writerow(headers)

        writer.writerow(data)
        print(f"File location: {filename}")

class subscriber(Node):
    def __init__(self):
        super().__init__("dual_leg_subscriber")
        timer_period = 0.5
        self.sub_l = self.create_subscription(Float64, "dual_leg_l", self.callback_l, 10)
        self.sub_r = self.create_subscription(Float64, "dual_leg_r", self.callback_r, 10)
        self.logger = self.create_timer(timer_period, self.callback_log)
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
    def callback_log(self):
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        data = [current_time, left_angle, right_angle]
        log_to_csv(CSV_PATH, data)
        print(f"Logged: {data}")

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