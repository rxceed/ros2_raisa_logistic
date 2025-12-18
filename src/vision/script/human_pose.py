#!/usr/bin/env python3

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
from std_msgs.msg import Bool

import threading
import queue
import time

class HumanPose(Node):

    def __init__(self):
        super().__init__('human_pose_node')

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
            '/vision/human_pose_frame',
            1
        )

        self.pub_zone = self.create_publisher(
            Int32,
            '/vision/human_zone',
            10
        )

        self.pub_zone_state = self.create_publisher(
            String,
            '/vision/human_zone_state',
            10
        )

        self.pub_hip_visible = self.create_publisher(
            Bool,
            '/vision/hip_visible',
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

        # ================= ZONE STATE =================
        self.prev_zone = None

        # ================= THREADING =================
        self.frame_queue = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self.inference_routine,
            daemon=True
        )
        self.thread.start()

    # ===================================================
    # ROS IMAGE CALLBACK
    # ===================================================
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Mirror + flip 180
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
            if results:
                self.publish_all(results, frame, header)
            else:
                self.pub_hip_visible.publish(Bool(data=False))

    # ===================================================
    # MEDIAPIPE PROCESSING
    # ===================================================
    def process_frame(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb)
        if not results.pose_landmarks:
            return None
        return results

    # ===================================================
    # ZONE COMPUTATION (LEFT -> RIGHT)
    # ===================================================
    def compute_active_zone(self, results, frame_width):
        nose = results.pose_landmarks.landmark[0]
        x_norm = nose.x  # 0.0 = kiri, 1.0 = kanan

        if x_norm < 0.25:
            return 0
        elif x_norm < 0.50:
            return 1
        elif x_norm < 0.75:
            return 2
        else:
            return 3
        
    def is_hip_visible(self, results, threshold=0.5):
        if not results or not results.pose_landmarks:
            return False
        
        landmarks = results.pose_landmarks.landmark
        left_hip = landmarks[23].visibility > threshold
        right_hip = landmarks[24].visibility > threshold
        
        return left_hip or right_hip

    # ===================================================
    # ZONE STATE MACHINE
    # ===================================================
    def publish_zone_state(self, zone):
        if self.prev_zone is None:
            self.prev_zone = zone
            return

        if zone != self.prev_zone:
            msg = String()
            msg.data = f"EXIT {self.prev_zone} -> ENTER {zone}"
            self.pub_zone_state.publish(msg)
            self.prev_zone = zone

    # ===================================================
    # HUD
    # ===================================================
    def draw_hud(self, frame):
        h, w, _ = frame.shape
        step = w // 4

        for i in range(1, 4):
            cv2.line(frame, (i * step, 0), (i * step, h), (255, 255, 255), 2)

        labels = ["ZONE 0", "ZONE 1", "ZONE 2", "ZONE 3"]
        for i, label in enumerate(labels):
            cv2.putText(
                frame,
                label,
                (i * step + 15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

    def draw_zone_label(self, frame, results, zone):
        nose = results.pose_landmarks.landmark[0]
        x = int(nose.x * frame.shape[1])
        y = int(nose.y * frame.shape[0]) - 20

        cv2.putText(
            frame,
            f"ZONE {zone}",
            (x, max(y, 30)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2
        )

    # ===================================================
    # PUBLISH ALL
    # ===================================================
    def publish_all(self, results, frame, header):
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
        self.publish_zone_state(zone)

        self.mp_draw.draw_landmarks(
            frame,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS
        )

        self.draw_hud(frame)
        self.draw_zone_label(frame, results, zone)

        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        img_msg.header = header
        img_msg.header.frame_id = "camera"
        self.pub_debug_image.publish(img_msg)

        hip_visible = self.is_hip_visible(results)
        self.pub_hip_visible.publish(Bool(data=hip_visible))

    # ===================================================
    # CLEANUP
    # ===================================================
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
