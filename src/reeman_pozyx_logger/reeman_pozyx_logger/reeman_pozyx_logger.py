import csv
from datetime import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import time
import os

FILENAME = f"data_log_{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}.csv"

CURRENT_DIR = os.path.dirname("./reeman_pozyx_logs/")
CSV_PATH = os.path.join(CURRENT_DIR, FILENAME)

def log_to_csv(filename, data):
    """
    Appends a list of data as a row to a CSV file.
    """
    headers = ['timestamp', 'pozyx_x', 'pozyx_y', 'pozyx_theta', 'reeman_x', 'reeman_y', 'reeman_theta']
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
        self.pozyx_x = 0
        self.pozyx_y = 0
        self.pozyx_theta = 0
        self.reeman_x = 0
        self.reeman_y = 0
        self.reeman_theta = 0
        super().__init__("reeman_pozyx_subscriber")
        timer_period = 0.3
        self.sub_pozyx = self.create_subscription(Pose2D, 'uwb_pose2d', self.callback_pozyx, 10)
        self.sub_reeman = self.create_subscription(Pose2D, 'reeman_pose2d', self.callback_reeman, 10)
        self.logger = self.create_timer(timer_period, self.callback_logger)
        self.sub_pozyx
        self.sub_reeman
    def callback_pozyx(self, msg):
        self.pozyx_x = msg.x
        self.pozyx_y = msg.y
        self.pozyx_theta = msg.theta
    def callback_reeman(self, msg):
        self.reeman_x = msg.x
        self.reeman_y = msg.y
        self.reeman_theta = msg.theta
    def callback_logger(self):
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        data = [current_time, self.pozyx_x, self.pozyx_y, self.pozyx_theta, self.reeman_x, self.reeman_y, self.reeman_theta]
        log_to_csv(CSV_PATH, data)
        print(f"Logged: {data}")

def main(arg=None):
    rclpy.init(args=arg)
    sub = subscriber()
    rclpy.spin(sub)
    try:
        while(1):
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()