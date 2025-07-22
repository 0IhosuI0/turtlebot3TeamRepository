import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import os
from pyzbar.pyzbar import decode
from ultralytics import YOLO
import numpy as np
import time

class QRCodeFollower(Node):
    def __init__(self):
        super().__init__('qr_code_follower')
        # ROS Communications
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.user_profile_pub = self.create_publisher(String, 'user_profile', 10)
        self.image_sub = self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # YOLO Model
        self.model = YOLO('yolov8n.pt')
        os.makedirs("users", exist_ok=True)

        # --- State Management ---
        self.robot_state = 'IDLE'  # IDLE, FOLLOWING, SEARCHING
        self.locked_target_id = None
        self.last_seen_box = None

        # Obstacle Avoidance
        self.safety_distance = 0.7 # Adjusted from 0.6
        self.critical_safety_distance = 0.3 # New: For very close obstacles, move backward
        self.lidar_zones = {'front': float('inf'), 'left': float('inf'), 'right': float('inf')}

        # Searching State
        self.search_state = 'PEEK'
        self.search_start_time = 0
        self.search_state_start_time = 0
        self.last_seen_side = 'CENTER'
        self.sweep_direction = 1
        self.lost_timeout = 15.0

    def scan_callback(self, msg):
        # Wider LiDAR zones for better obstacle detection
        # Right zone (270-330 degrees)
        self.lidar_zones['right'] = min([r for r in msg.ranges[270:330] if r > 0.01] or [float('inf')])
        # Front zone (330-360 and 0-30 degrees)
        self.lidar_zones['front'] = min([r for r in (msg.ranges[330:360] + msg.ranges[0:30]) if r > 0.01] or [float('inf')])
        # Left zone (30-90 degrees)
        self.lidar_zones['left']  = min([r for r in msg.ranges[30:90] if r > 0.01] or [float('inf')])

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        frame_height, frame_width = frame.shape[:2]
        results = self.model(frame, verbose=False)[0]
        person_boxes = [list(map(int, box.xyxy[0])) for box in results.boxes if self.model.names[int(box.cls[0])] == 'person']

        target_visible_this_frame = False
        current_target_box = None

        # --- Target Detection & State Update ---
        if self.locked_target_id is not None:
            # If locked, try to re-acquire the target
            if person_boxes:
                last_center_x = (self.last_seen_box[0] + self.last_seen_box[2]) / 2
                potential_matches = []
                for p_box in person_boxes:
                    p_center_x = (p_box[0] + p_box[2]) / 2
                    if abs(p_center_x - last_center_x) < 150:
                        potential_matches.append((abs(p_center_x - last_center_x), p_box))
                if potential_matches:
                    _, current_target_box = min(potential_matches, key=lambda item: item[0])
                    target_visible_this_frame = True
        else:
            # If not locked (IDLE), try to find a new target via QR code
            qr_codes = decode(frame)
            for qr in qr_codes:
                try:
                    user_info = json.loads(qr.data.decode("utf-8"))
                    user_id = user_info.get('user_id')
                    x, y, w, h = qr.rect
                    qr_center_x, qr_center_y = x + w // 2, y + h // 2
                    for p_box in person_boxes:
                        if p_box[0] < qr_center_x < p_box[2] and p_box[1] < qr_center_y < p_box[3]:
                            self.get_logger().info(f'New target locked: {user_id}')
                            self.locked_target_id = user_id
                            current_target_box = p_box
                            target_visible_this_frame = True
                            self.save_user_profile(user_info)
                            self.user_profile_pub.publish(String(data=json.dumps(user_info)))
                            break
                except Exception as e:
                    self.get_logger().error(f'QR parsing failed: {e}')
                if target_visible_this_frame: break

        # --- State Transition ---
        if target_visible_this_frame:
            self.robot_state = 'FOLLOWING'
            self.last_seen_box = current_target_box
            target_center_x = (current_target_box[0] + current_target_box[2]) / 2
            if target_center_x < frame_width / 3: self.last_seen_side = 'LEFT'
            elif target_center_x > frame_width * 2 / 3: self.last_seen_side = 'RIGHT'
            else: self.last_seen_side = 'CENTER'
        else:
            if self.robot_state == 'FOLLOWING':
                self.get_logger().info("Target lost. Starting search.")
                self.robot_state = 'SEARCHING'
                self.search_state = 'PEEK'
                self.search_start_time = time.time()
                self.search_state_start_time = time.time()

        # --- Hierarchical Action Selection ---
        final_twist = self.generate_motion_command(current_target_box, frame_width)
        self.cmd_vel_pub.publish(final_twist)

        # --- Visualization ---
        if current_target_box:
            px1, py1, px2, py2 = current_target_box
            label = f"Target: {self.locked_target_id}"
            cv2.rectangle(frame, (px1, py1), (px2, py2), (0, 255, 255), 3)
            cv2.putText(frame, label, (px1, py1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("QR Code Follower", frame)
        cv2.waitKey(1)

    def generate_motion_command(self, target_box, frame_width):
        # 0. Critical Obstacle Avoidance (Highest Priority - Move Backward)
        # If any zone is within critical safety distance, move backward
        if (self.lidar_zones['front'] < self.critical_safety_distance or
            self.lidar_zones['left'] < self.critical_safety_distance or
            self.lidar_zones['right'] < self.critical_safety_distance):
            self.get_logger().warn("CRITICAL OBSTACLE! Moving backward.")
            twist_cmd = Twist()
            twist_cmd.linear.x = -0.1 # Move backward
            twist_cmd.angular.z = 0.0 # No rotation while backing up
            return twist_cmd

        # 1. Obstacle Avoidance Layer (High Priority - Front)
        if self.lidar_zones['front'] < self.safety_distance:
            self.get_logger().warn("Obstacle FRONT! Turning.")
            twist_cmd = Twist()
            twist_cmd.linear.x = 0.0
            # Turn towards the more open side, more aggressively
            twist_cmd.angular.z = -1.0 if self.lidar_zones['left'] < self.lidar_zones['right'] else 1.0
            return twist_cmd

        # 2. State-Based Action Layer
        # Generate the base movement command based on robot state
        if self.robot_state == 'FOLLOWING':
            base_twist = self.get_following_twist(target_box, frame_width)
        elif self.robot_state == 'SEARCHING':
            base_twist = self.get_searching_twist()
        else: # IDLE
            base_twist = Twist()

        # 3. Side Obstacle Avoidance Layer (Modifies base_twist)
        # If a side obstacle is detected, adjust angular and linear velocity
        side_obstacle_detected = False
        if self.lidar_zones['left'] < self.safety_distance:
            self.get_logger().warn("Obstacle LEFT! Adjusting motion to turn right.")
            base_twist.angular.z -= 0.7 # Adjusted from 0.9
            side_obstacle_detected = True
        if self.lidar_zones['right'] < self.safety_distance:
            self.get_logger().warn("Obstacle RIGHT! Adjusting motion to turn left.")
            base_twist.angular.z += 0.7 # Adjusted from 0.9
            side_obstacle_detected = True

        if side_obstacle_detected:
            # If a side obstacle is detected, significantly reduce forward speed
            # but allow a very small forward motion to keep maneuvering.
            # Or, if already reversing, maintain that.
            if base_twist.linear.x > 0: # If currently moving forward
                base_twist.linear.x = min(base_twist.linear.x, 0.1) # Limit to a small positive speed (Increased from 0.05)
            elif base_twist.linear.x < 0: # If currently reversing
                # Keep reversing, but ensure it's not too fast
                base_twist.linear.x = max(base_twist.linear.x, -0.1) # Limit to a small negative speed (Increased from -0.05)
            # If linear.x is 0, keep it 0.

        return base_twist

    def get_following_twist(self, target_box, frame_width):
        twist_msg = Twist()
        x1, y1, x2, y2 = target_box
        target_center_x = (x1 + x2) // 2
        target_area = (x2 - x1) * (y2 - y1)
        error_x = target_center_x - frame_width // 2
        twist_msg.angular.z = -0.003 * error_x # Increased from 0.002
        if target_area < 60000: twist_msg.linear.x = 0.15
        elif target_area > 120000: twist_msg.linear.x = -0.15
        else: twist_msg.linear.x = 0.0
        return twist_msg

    def get_searching_twist(self):
        if time.time() - self.search_start_time > self.lost_timeout:
            self.get_logger().info("Search timeout. Returning to IDLE state.")
            self.robot_state = 'IDLE'
            self.locked_target_id = None
            return Twist()

        twist_msg = Twist()
        current_time = time.time()
        if self.search_state == 'PEEK':
            if current_time - self.search_state_start_time > 1.5:
                self.search_state = 'SWEEP'
                self.search_state_start_time = current_time
            else:
                twist_msg.angular.z = -0.4 if self.last_seen_side == 'RIGHT' else 0.4
        elif self.search_state == 'SWEEP':
            if current_time - self.search_state_start_time > 3.0:
                self.sweep_direction *= -1
                self.search_state_start_time = current_time
            twist_msg.angular.z = 0.6 * self.sweep_direction # Increased from 0.4
        return twist_msg

    def save_user_profile(self, user_data):
        user_id = user_data.get("user_id", "unknown")
        with open(f"users/{user_id}.json", "w") as f:
            json.dump(user_data, f, indent=4)
        self.get_logger().info(f'User profile saved: users/{user_id}.json')

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
