#!/usr/bin/env python3

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32
from cv_bridge import CvBridge

import threading
import queue
import time
import os
import csv
from datetime import datetime


class PoseDataLogger(Node):

    def __init__(self):
        super().__init__('pose_data_logger')

        # ================= ACTIVITY LABEL =================
        self.current_activity = "standing"

        self.activity_map = {
            ord('1'): "standing",
            ord('2'): "walking",
            ord('3'): "sitting",
            ord('4'): "picking",
            ord('5'): "waving",
        }

        # ================= ROS INTERFACES =================
        self.sub_image = self.create_subscription(
            Image,
            '/vision/image_raw',
            self.image_callback,
            10
        )

        self.pub_pose = self.create_publisher(
            PoseArray,
            '/vision/human_pose',
            10
        )

        self.pub_debug_image = self.create_publisher(
            Image,
            '/vision/pose_logger_frame',
            1
        )

        self.pub_zone = self.create_publisher(
            Int32,
            '/vision/human_zone',
            10
        )

        # ================= CV / MEDIAPIPE =================
        self.bridge = CvBridge()
        self.mp_pose = mp.solutions.pose
        self.mp_draw = mp.solutions.drawing_utils

        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # ================= DATASET LOGGER =================
        self.dataset_dir = os.path.expanduser(
            "./skeleton_data_logs/"
        )
        os.makedirs(self.dataset_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.csv_path = os.path.join(
            self.dataset_dir,
            f"pose_logger_{self.current_activity}_{timestamp}.csv"
        )

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ["timestamp", "zone", "activity", "joint", "x", "y", "z", "confidence"]
        )

        self.get_logger().info("Pose Data Logger started")
        self.get_logger().info("Hotkeys: [1-5]=activity | [q]=quit")
        self.get_logger().info(f"CSV output: {self.csv_path}")

        # ================= THREADING =================
        self.frame_queue = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self.inference_routine,
            daemon=True
        )
        self.thread.start()

    # ===================================================
    # IMAGE CALLBACK
    # ===================================================
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Mirror + Flip 180
            frame = cv2.flip(frame, 1)
            frame = cv2.flip(frame, -1)

            if not self.frame_queue.full():
                self.frame_queue.put((frame, msg.header))

        except Exception as e:
            self.get_logger().error(str(e))

    # ===================================================
    # INFERENCE THREAD
    # ===================================================
    def inference_routine(self):
        while not self.stop_event.is_set():
            if self.frame_queue.empty():
                time.sleep(0.005)
                continue

            frame, header = self.frame_queue.get()
            results = self.process_frame(frame)
            if results is not None:
                self.publish_and_log(results, frame, header)

    # ===================================================
    # MEDIAPIPE
    # ===================================================
    def process_frame(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb)
        if not results.pose_landmarks:
            return None
        return results

    # ===================================================
    # HUD UTILS
    # ===================================================
    def compute_active_zone(self, results, frame_width, segments=4):
        nose = results.pose_landmarks.landmark[0]
        x_px = int(nose.x * frame_width)
        zone = int((x_px / frame_width) * segments)
        return min(max(zone, 0), segments - 1)

    def draw_hud(self, frame, zone):
        h, w, _ = frame.shape
        step = w // 4

        for i in range(1, 4):
            cv2.line(frame, (i * step, 0), (i * step, h), (255, 255, 255), 2)

        text = f"ACTIVITY: {self.current_activity}"
        (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)

        cv2.putText(
            frame,
            text,
            (w - tw - 20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"ZONE {zone}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 0),
            2
        )

    # ===================================================
    # HOTKEY
    # ===================================================
    def handle_hotkeys(self):
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            self.get_logger().info("Shutdown requested")
            rclpy.shutdown()

        elif key in self.activity_map:
            new_activity = self.activity_map[key]
            if new_activity != self.current_activity:
                self.current_activity = new_activity
                self.get_logger().info(f"Activity changed to: {self.current_activity}")

    # ===================================================
    # LOG + PUBLISH
    # ===================================================
    def publish_and_log(self, results, frame, header):
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.header.frame_id = "camera"

        for lm in results.pose_landmarks.landmark:
            p = Pose()
            p.position.x = lm.x
            p.position.y = lm.y
            p.position.z = lm.z
            p.orientation.w = lm.visibility
            pose_array.poses.append(p)

        self.pub_pose.publish(pose_array)

        zone = self.compute_active_zone(results, frame.shape[1])
        self.pub_zone.publish(Int32(data=zone))

        t = header.stamp.sec + header.stamp.nanosec * 1e-9

        for idx, lm in enumerate(results.pose_landmarks.landmark):
            joint = self.mp_pose.PoseLandmark(idx).name.lower()
            self.csv_writer.writerow([
                t, zone, self.current_activity,
                joint, lm.x, lm.y, lm.z, lm.visibility
            ])

        self.mp_draw.draw_landmarks(
            frame,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS
        )

        self.draw_hud(frame, zone)
        self.handle_hotkeys()

        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        img_msg.header = header
        self.pub_debug_image.publish(img_msg)

    # ===================================================
    # CLEANUP
    # ===================================================
    def destroy_node(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.csv_file.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
