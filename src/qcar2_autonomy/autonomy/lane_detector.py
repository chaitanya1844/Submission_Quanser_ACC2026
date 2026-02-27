#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import numpy as np
import torch
import torch.nn as nn
import os
import time
import math
import cv2
import subprocess

from tf2_ros import Buffer, TransformListener


# ======================
# MODEL
# ======================
class DoubleConv(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(out_ch, out_ch, 3, padding=1),
            nn.ReLU()
        )

    def forward(self, x):
        return self.net(x)


class UNet(nn.Module):
    def __init__(self):
        super().__init__()

        self.d1 = DoubleConv(3, 32)
        self.p1 = nn.MaxPool2d(2)

        self.d2 = DoubleConv(32, 64)
        self.p2 = nn.MaxPool2d(2)

        self.bridge = DoubleConv(64, 128)

        self.u1 = nn.ConvTranspose2d(128, 64, 2, stride=2)
        self.d3 = DoubleConv(128, 64)

        self.u2 = nn.ConvTranspose2d(64, 32, 2, stride=2)
        self.d4 = DoubleConv(64, 32)

        self.out = nn.Conv2d(32, 1, 1)

    def forward(self, x):
        d1 = self.d1(x)
        d2 = self.d2(self.p1(d1))
        bridge = self.bridge(self.p2(d2))

        u1 = self.u1(bridge)
        u1 = torch.cat([u1, d2], dim=1)
        d3 = self.d3(u1)

        u2 = self.u2(d3)
        u2 = torch.cat([u2, d1], dim=1)
        d4 = self.d4(u2)

        return torch.sigmoid(self.out(d4))


# ======================
# NODE
# ======================
class LaneDetector(Node):

    def __init__(self):

        super().__init__('lane_detector')

        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.create_subscription(
            Image,
            '/camera/color_image',
            self.image_callback,
            10
        )

        # STOP SIGN
        self.motion_allowed = True
        self.stop_until = 0.0

        self.create_subscription(
            Bool,
            '/motion_enable',
            self.motion_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ======================
        # WAYPOINTS
        # ======================
        self.pickup_point = (-1.605, 5.285)

        self.dropoff_path = [
            (-2.489, 2.095), (-2.256, 1.782), (-2.065, 1.572),
            (-1.726, 1.356), (-1.503, 1.282), (-1.238, 1.285),
            (-0.920, 1.343), (-0.832, 1.433), (-0.447, 1.691),
            (0.185, 2.121), (0.674, 2.437), (0.759, 2.510),
            (0.902, 2.606), (1.183, 2.778), (1.633, 3.100),
            (1.679, 3.320), (1.658, 3.536), (1.598, 3.727),
            (1.531, 3.795)
        ]

        self.dropoff_point = (-0.714,1.469)
        self.taxi_hub_point = (-0.124,-0.020)

        self.lookahead = 0.6
        self.base_speed = 0.22

        # ======================
        # STATE
        # ======================
        self.mode = "LANE"
        self.prev_mode = None

        self.pickup_wait_start = None
        self.drop_wait_start = None

        self.pickup_done = False
        self.drop_stop_done = False

        self.after_pickup = False  # ⭐ keep LED blue after pickup

        self.offset = 55

        # LED START
        self.set_led(5)

        # Visualization window
        cv2.namedWindow("Lane + Sign View", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Lane + Sign View", 400, 300)

        # ======================
        # MODEL
        # ======================
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        self.model = UNet().to(self.device)

        model_path = os.path.expanduser(
            '/home/chaitanya/Documents/ACC_Development/Development/ros2/src/qcar2_autonomy/models/lane_segmentation.pth'
        )

        if os.path.exists(model_path):
            self.model.load_state_dict(
                torch.load(model_path, map_location=self.device)
            )
            self.get_logger().info("Model loaded")

        self.model.eval()

    # ======================
    # LED
    # ======================
    def set_led(self, color_id):
        try:
            subprocess.Popen([
                "ros2", "param", "set",
                "qcar2_hardware",
                "led_color_id",
                str(color_id)
            ])
        except:
            pass

    # ======================
    # STOP SIGN
    # ======================
    def motion_callback(self, msg):

        if not msg.data:
            self.motion_allowed = False
            self.stop_until = time.time() + 10.0
        else:
            self.motion_allowed = True

    # ======================
    # POSE
    # ======================
    def get_pose(self):

        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y

            q = t.transform.rotation
            yaw = self.quaternion_to_yaw(q)

            return x, y, yaw

        except:
            return None, None, None

    def quaternion_to_yaw(self, q):

        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)

        return math.atan2(siny, cosy)

    # ======================
    # PURE PURSUIT
    # ======================
    def pure_pursuit(self, curr_x, curr_y, curr_yaw, path):

        dists = [math.hypot(px - curr_x, py - curr_y) for px, py in path]
        closest_i = int(np.argmin(dists))

        target = path[-1]

        for i in range(closest_i, len(path)):
            px, py = path[i]
            if math.hypot(px - curr_x, py - curr_y) > self.lookahead:
                target = (px, py)
                break

        tx, ty = target

        dx = tx - curr_x
        dy = ty - curr_y

        local_x = math.cos(curr_yaw) * dx + math.sin(curr_yaw) * dy
        local_y = -math.sin(curr_yaw) * dx + math.cos(curr_yaw) * dy

        if abs(local_x) < 1e-6:
            return 0.0, 0.0, closest_i

        curvature = 2 * local_y / (self.lookahead ** 2)

        steering = curvature * 0.9
        steering = max(min(steering, 0.25), -0.25)

        throttle = self.base_speed - 0.1 * abs(steering)

        return throttle, steering, closest_i

    # ======================
    # CALLBACK
    # ======================
    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        if frame is None:
            return

        curr_x, curr_y, curr_yaw = self.get_pose()
        if curr_x is None:
            return

        steering = 0.0
        throttle = 0.0

        # ======================
        # STATE LED CHANGES
        # ======================
        if self.mode != self.prev_mode:

            if self.mode == "LANE":

                if self.after_pickup:
                    self.set_led(2)  # stay blue
                else:
                    self.set_led(1)

            elif self.mode == "PICKUP_STOP":
                self.set_led(2)

            elif self.mode == "DROP_WAIT":
                self.set_led(3)

            elif self.mode == "FINAL_STOP":
                self.set_led(5)

            self.prev_mode = self.mode

        # ======================
        # LANE FOLLOW
        # ======================
        if self.mode == "LANE":

            steering, _ = self.compute_steering(frame)
            throttle = 0.3
            px, py = self.pickup_point
            if (not self.pickup_done and
                math.hypot(px - curr_x, py - curr_y) < 0.35):

                self.mode = "PICKUP_STOP"
                self.pickup_wait_start = time.time()

            first_tx, first_ty = self.dropoff_path[0]
            if math.hypot(first_tx - curr_x, first_ty - curr_y) < 0.7:
                self.mode = "CURVE_PATH"

        elif self.mode == "PICKUP_STOP":

            throttle = 0.0
            steering = 0.0

            if time.time() - self.pickup_wait_start > 3.0:
                self.pickup_done = True
                self.after_pickup = True
                self.mode = "LANE"

        elif self.mode == "CURVE_PATH":

            throttle, steering, idx = self.pure_pursuit(
                curr_x, curr_y, curr_yaw,
                self.dropoff_path
            )

            drop_x, drop_y = self.dropoff_point
            dist_to_drop = math.hypot(curr_x - drop_x, curr_y - drop_y)

            if (not self.drop_stop_done and dist_to_drop < 0.35):

                self.mode = "DROP_WAIT"
                self.drop_wait_start = time.time()
                return

            if idx >= len(self.dropoff_path) - 2:
                self.mode = "LANE_RESUME"

        elif self.mode == "DROP_WAIT":

            throttle = 0.0
            steering = 0.0

            if time.time() - self.drop_wait_start > 5.0:
                self.drop_stop_done = True
                self.mode = "CURVE_PATH"

        elif self.mode == "LANE_RESUME":

            steering, _ = self.compute_steering(frame)
            throttle = 0.25

            tx, ty = self.taxi_hub_point

            if math.hypot(curr_x - tx, curr_y - ty) < 0.35:
                self.mode = "FINAL_STOP"

        elif self.mode == "FINAL_STOP":

            throttle = 0.0
            steering = 0.0

        if not self.motion_allowed:
            if time.time() < self.stop_until:
                throttle = 0.0
            else:
                self.motion_allowed = True

        cmd = Twist()
        cmd.linear.x = float(throttle)
        cmd.angular.z = float(steering)
        self.cmd_pub.publish(cmd)

    # ======================
    # LANE
    # ======================
    def compute_steering(self, frame):

        h, w, _ = frame.shape
        y1 = int(h*0.5)
        y2 = int(h*0.9)

        roi = frame[y1:y2, :]

        img = cv2.resize(roi, (160, 80))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        tensor = torch.from_numpy(img).float() / 255.0
        tensor = tensor.permute(2, 0, 1).unsqueeze(0).to(self.device)

        with torch.no_grad():
            pred = self.model(tensor)[0, 0].cpu().numpy()

        mask = cv2.resize(pred, (roi.shape[1], roi.shape[0]))
        binary = (mask > 0.5).astype(np.uint8)

        self.visualize(frame, binary, y1, y2)

        targets = []

        for row in range(int(binary.shape[0]*0.4),
                         binary.shape[0], 4):

            cols = np.where(binary[row] > 0)[0]

            if len(cols) < 30:
                continue

            right = cols[-1]
            target = max(0, right - self.offset)

            targets.append(target)

        if len(targets) == 0:
            return 0.0, 0.0

        lane_target = int(np.mean(targets))
        image_center = int(binary.shape[1] * 0.5)

        error = (lane_target - image_center) / image_center
        steer = -0.4 * error

        return steer, float(error)

    def visualize(self, frame, binary_mask, y1, y2):
    
        try:
            overlay = frame.copy()
    
            # Create color mask same size as ROI
            roi_h = y2 - y1
            roi_w = frame.shape[1]
    
            mask_resized = cv2.resize(binary_mask, (roi_w, roi_h))
    
            mask_color = np.zeros((roi_h, roi_w, 3), dtype=np.uint8)
            mask_color[:, :, 1] = mask_resized * 255   # green lane
    
            # Overlay only on ROI
            overlay[y1:y2, :] = cv2.addWeighted(
                overlay[y1:y2, :],
                1.0,
                mask_color,
                0.5,
                0
            )
    
            # Draw center reference line
            h, w, _ = overlay.shape
            cv2.line(overlay, (w//2, h), (w//2, int(h*0.6)), (255, 0, 0), 2)
    
            # Resize for display window
            display = cv2.resize(overlay, (420, 300))
    
            cv2.imshow("Lane + Sign View", display)
            cv2.waitKey(1)
    
        except Exception as e:
            print("Visualization error:", e)

def main():

    rclpy.init()

    node = LaneDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()