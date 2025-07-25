import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
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

from museum_robot_msgs.action import FollowUser

class QRCodeFollower(Node):
    def __init__(self):
        super().__init__('qr_code_follower')
        self.get_logger().info("QRCodeFollower node initializing...")
        # ROS Communications
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.qr_detection_pub = self.create_publisher(String, '/museum/qr_detected', 10)
        self.image_sub = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        )
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Action Server
        self._action_server = ActionServer(
            self,
            FollowUser,
            'follow_user',
            self.execute_callback
        )
        self.get_logger().info("Action server created.")

        # YOLO Model
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info("YOLO model loaded.")
        
        # --- State Management ---
        self.robot_state = 'IDLE'  # IDLE, FOLLOWING, SEARCHING, OBSTACLE_AVOIDANCE
        self.locked_target_id = None
        self.last_seen_box = None
        self.current_goal_handle = None

        # Obstacle Avoidance
        self.safety_distance = 0.7
        self.critical_safety_distance = 0.3
        self.lidar_zones = {'front': float('inf'), 'left': float('inf'), 'right': float('inf')}

        # Searching State
        self.search_state = 'PEEK'
        self.search_start_time = 0
        self.search_state_start_time = 0
        self.last_seen_side = 'CENTER'
        self.sweep_direction = 1
        self.lost_timeout = 15.0
        cv2.namedWindow("QR Code Follower")
        self.get_logger().info("QRCodeFollower node initialized.")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        command = goal_handle.request.command
        user_id = goal_handle.request.user_id
        
        if command == "START":
            self.robot_state = 'SEARCHING'
            self.locked_target_id = user_id
            self.current_goal_handle = goal_handle
            self.get_logger().info(f'Starting to follow user: {user_id}')
            
            feedback_msg = FollowUser.Feedback()
            while self.robot_state != 'IDLE':
                feedback_msg.current_state = self.robot_state
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)
            
            goal_handle.succeed()
            result = FollowUser.Result()
            result.status = "Successfully stopped following."
            return result
            
        elif command == "STOP":
            self.robot_state = 'IDLE'
            self.locked_target_id = None
            self.cmd_vel_pub.publish(Twist()) # 로봇 정지
            if self.current_goal_handle:
                self.current_goal_handle.succeed()
            goal_handle.succeed()
            result = FollowUser.Result()
            result.status = "Stopped."
            return result
        else:
            goal_handle.abort()
            result = FollowUser.Result()
            result.status = "Invalid command."
            return result

    def scan_callback(self, msg):
        self.get_logger().debug("Scan callback received.") # Changed to debug
        self.lidar_zones['right'] = min([r for r in msg.ranges[270:330] if r > 0.01] or [float('inf')])
        self.lidar_zones['front'] = min([r for r in (msg.ranges[330:360] + msg.ranges[0:30]) if r > 0.01] or [float('inf')])
        self.lidar_zones['left']  = min([r for r in msg.ranges[30:90] if r > 0.01] or [float('inf')])

    def image_callback(self, msg):
        self.get_logger().info("Image callback entered.")
        if self.robot_state == 'IDLE':
            self.get_logger().info(f"Robot state is IDLE, skipping image processing. Current state: {self.robot_state}")
            return
            
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.get_logger().debug("Image decoded.") # Changed to debug
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        frame_height, frame_width = frame.shape[:2]
        results = self.model(frame, verbose=False)[0]
        person_boxes = [list(map(int, box.xyxy[0])) for box in results.boxes if self.model.names[int(box.cls[0])] == 'person']
        self.get_logger().debug(f"Detected {len(person_boxes)} persons.") # Changed to debug

        target_visible_this_frame = False
        current_target_box = None

        if self.locked_target_id is not None:
            if self.robot_state == 'FOLLOWING':
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
            elif self.robot_state == 'SEARCHING':
                qr_codes = decode(frame)
                self.get_logger().debug(f"Detected {len(qr_codes)} QR codes.") # Changed to debug
                for qr in qr_codes:
                    try:
                        user_info = json.loads(qr.data.decode("utf-8"))
                        if user_info.get('user_id') == self.locked_target_id:
                            x, y, w, h = qr.rect
                            qr_center_x, qr_center_y = x + w // 2, y + h // 2
                            for p_box in person_boxes:
                                if p_box[0] < qr_center_x < p_box[2] and p_box[1] < qr_center_y < p_box[3]:
                                    self.get_logger().info(f'Re-acquired target: {self.locked_target_id}')
                                    current_target_box = p_box
                                    target_visible_this_frame = True
                                    self.qr_detection_pub.publish(String(data=json.dumps({"user_id": self.locked_target_id})))
                                    break
                    except Exception as e:
                        self.get_logger().error(f'QR parsing failed: {e}')
                    if target_visible_this_frame: break

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

        final_twist = self.generate_motion_command(current_target_box, frame_width)
        self.cmd_vel_pub.publish(final_twist)
        self.get_logger().debug(f"Published Twist command: linear.x={final_twist.linear.x}, angular.z={final_twist.angular.z}") # Changed to debug

        if current_target_box:
            px1, py1, px2, py2 = current_target_box
            label = f"Target: {self.locked_target_id}"
            cv2.rectangle(frame, (px1, py1), (px2, py2), (0, 255, 255), 3)
            cv2.putText(frame, label, (px1, py1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("QR Code Follower", frame)
        cv2.waitKey(1)
        self.get_logger().debug("Image displayed.") # Changed to debug

    def generate_motion_command(self, target_box, frame_width):
        self.get_logger().debug("Generating motion command.") # Changed to debug
        if self.lidar_zones['front'] < self.critical_safety_distance or \
           self.lidar_zones['left'] < self.critical_safety_distance or \
           self.lidar_zones['right'] < self.critical_safety_distance:
            self.robot_state = 'OBSTACLE_AVOIDANCE'
            self.get_logger().warn("CRITICAL OBSTACLE! Moving backward.")
            twist_cmd = Twist()
            twist_cmd.linear.x = -0.1
            return twist_cmd

        if self.lidar_zones['front'] < self.safety_distance:
            self.robot_state = 'OBSTACLE_AVOIDANCE'
            self.get_logger().warn("Obstacle FRONT! Turning.")
            twist_cmd = Twist()
            twist_cmd.angular.z = -1.0 if self.lidar_zones['left'] < self.lidar_zones['right'] else 1.0
            return twist_cmd

        if self.robot_state == 'FOLLOWING':
            base_twist = self.get_following_twist(target_box, frame_width)
        elif self.robot_state == 'SEARCHING':
            base_twist = self.get_searching_twist()
        else:
            base_twist = Twist()

        side_obstacle_detected = False
        if self.lidar_zones['left'] < self.safety_distance:
            self.get_logger().warn("Obstacle LEFT! Adjusting motion to turn right.")
            base_twist.angular.z -= 0.7
            side_obstacle_detected = True
        if self.lidar_zones['right'] < self.safety_distance:
            self.get_logger().warn("Obstacle RIGHT! Adjusting motion to turn left.")
            base_twist.angular.z += 0.7
            side_obstacle_detected = True

        if side_obstacle_detected:
            self.robot_state = 'OBSTACLE_AVOIDANCE'
            if base_twist.linear.x > 0:
                base_twist.linear.x = min(base_twist.linear.x, 0.1)
            elif base_twist.linear.x < 0:
                base_twist.linear.x = max(base_twist.linear.x, -0.1)
        
        return base_twist

    def get_following_twist(self, target_box, frame_width):
        self.get_logger().debug("Getting following twist.") # Changed to debug
        twist_msg = Twist()
        if target_box is None:
            return twist_msg
        x1, y1, x2, y2 = target_box
        target_center_x = (x1 + x2) // 2
        target_area = (x2 - x1) * (y2 - y1)
        error_x = target_center_x - frame_width // 2
        twist_msg.angular.z = -0.003 * error_x
        if target_area < 60000:
            twist_msg.linear.x = 0.15
        elif target_area > 120000:
            twist_msg.linear.x = -0.15
        else:
            twist_msg.linear.x = 0.0
        return twist_msg

    def get_searching_twist(self):
        self.get_logger().debug("Getting searching twist.") # Changed to debug
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
            twist_msg.angular.z = 0.6 * self.sweep_direction
        return twist_msg

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()