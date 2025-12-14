#!/usr/bin/env python3

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge

import threading
import queue
import time


class HumanPose(Node):

    def __init__(self):
        super().__init__('human_pose_node')

        # === ROS INTERFACES ===
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
            '/vision/human_pose_frame',
            1
        )

        # === CV / MEDIAPIPE ===
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

        # === THREADING ===
        self.frame_queue = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self.inference_routine,
            daemon=True
        )
        self.thread.start()

        self.get_logger().info("HumanPose Node Active")

    # =====================================================
    # ROS CALLBACK
    # =====================================================
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if not self.frame_queue.full():
                self.frame_queue.put((frame, msg.header))
        except Exception as e:
            self.get_logger().error(str(e))

    # =====================================================
    # INFERENCE THREAD
    # =====================================================
    def inference_routine(self):
        while not self.stop_event.is_set():
            if self.frame_queue.empty():
                time.sleep(0.005)
                continue

            frame, header = self.frame_queue.get()
            results = self.process_frame(frame)
            if results is not None:
                self.publish_pose(results, frame)

    # =====================================================
    # CORE PROCESSING
    # =====================================================
    def process_frame(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb)

        if not results.pose_landmarks:
            return None

        return results

    # =====================================================
    # PUBLISHERS (POSE + IMAGE)
    # =====================================================
    def publish_pose(self, results, frame):
        # ---- PoseArray ----
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera"

        for lm in results.pose_landmarks.landmark:
            p = Pose()
            p.position.x = lm.x
            p.position.y = lm.y
            p.position.z = lm.z
            p.orientation.w = lm.visibility
            pose_array.poses.append(p)

        self.pub_pose.publish(pose_array)

        # ---- Draw landmarks ----
        self.mp_draw.draw_landmarks(
            frame,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS
        )

        # ---- Publish Image for web_video_server ----
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera"
        self.pub_debug_image.publish(img_msg)

    # =====================================================
    # CLEANUP
    # =====================================================
    def destroy_node(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
