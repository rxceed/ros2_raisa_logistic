import rclpy
from rclpy.node import Node
import numpy as np
import tensorflow as tf
import joblib
from collections import deque

from custom_interface.msg import DualLeg, ActivityForecast
from geometry_msgs.msg import Pose2D


# ==========================================================
# CONFIG
# ==========================================================

WINDOW_SIZE = 10
FORECAST_STEPS = 4

FEATURE_ORDER = [
    "left_leg_angle",
    "right_leg_angle",
    "x_relative",
    "y_relative",
    "x_absolute",
    "y_absolute"
]

ACTION_MAP = {
    0: "sitting",
    1: "standing",
    2: "walking",
    3: "pre-standing",
    4: "pre-sitting"
}


# ==========================================================
# ROS2 NODE
# ==========================================================

class LSTMInferenceNode(Node):
    def __init__(self):
        super().__init__("lstm_inference_node")
        self.left_leg_angle = 0
        self.right_leg_angle = 0
        self.x_abs = 0
        self.y_abs = 0
        self.x_before = 0
        self.y_before = 0
        self.x_rel = 0
        self.y_rel = 0
        
        self.model = tf.keras.models.load_model("model/LSTM.keras")
        self.scaler = joblib.load("model/feature_scaler.save")

        self.buffer = deque(maxlen=WINDOW_SIZE)

        self.sub_dualLeg = self.create_subscription(DualLeg, "dual_leg", self.callback_sub_dualLeg, 10)
        self.sub_uwb = self.create_subscription(Pose2D,"uwb_pose2d", self.callback_sub_uwb, 10)

        self.pub = self.create_publisher(ActivityForecast, "activity_forecast", 10)

        timer_period = 1
        self.timer = self.create_timer(timer_period, callback=self.inference_callback)
    
    def callback_sub_dualLeg(self, msg):
        self.left_leg_angle = msg.right_angle
        self.right_leg_angle = msg.right_angle

    def callback_sub_uwb(self, msg):
        self.x_abs = msg.x
        self.y_abs = msg.y
        self.x_rel = abs(self.x_before - self.x_abs)
        self.y_rel = abs(self.y_before - self.y_abs)
        self.x_before = self.x_abs
        self.y_before = self.y_abs

    def inference_callback(self):
        features = np.array([
            self.left_leg_angle,
            self.right_leg_angle,
            self.x_rel,
            self.y_rel,
            self.x_abs,
            self.y_abs
        ]).reshape(1, -1)

        features = self.scaler.transform(features)[0]
        self.buffer.append(features)

        if len(self.buffer) < WINDOW_SIZE:
            return

        X = np.expand_dims(np.array(self.buffer), axis=0)
        preds = self.model.predict(X, verbose=0)

        forecast = ActivityForecast()

        # ---- Action forecasting ----
        for i in range(FORECAST_STEPS):
            probs = preds[i][0]
            action_id = int(np.argmax(probs))

            forecast.actions.append(ACTION_MAP[action_id])
            forecast.confidences.append(float(probs[action_id]))

        # ---- Time-to-arrival ----
        forecast.time_to_arrival = float(preds[-1][0][0])
        self.pub.publish(forecast)

# ==========================================================
# MAIN
# ==========================================================

def main():
    rclpy.init()
    node = LSTMInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
