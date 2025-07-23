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

# --- Constants ---
# General
USER_DATA_DIR = "users"
YOLO_MODEL_PATH = 'yolov8n.pt'

# Node and Topic Names
NODE_NAME = 'qr_code_follower'
CMD_VEL_TOPIC = '/cmd_vel'
USER_PROFILE_TOPIC = 'user_profile'
IMAGE_TOPIC = 'camera/image_raw/compressed'
SCAN_TOPIC = '/scan'

# Robot Behavior Parameters
SAFETY_DISTANCE = 0.7  # meters
CRITICAL_SAFETY_DISTANCE = 0.3  # meters
LOST_TIMEOUT = 15.0  # seconds

# Following Control
ANGULAR_KP = -0.003  # Proportional gain for angular velocity
MIN_TARGET_AREA = 15000  # pixels^2 (Adjusted for color tracking box)
MAX_TARGET_AREA = 90000  # pixels^2 (Adjusted for color tracking box)
FORWARD_SPEED = 0.15  # m/s
BACKWARD_SPEED = -0.15  # m/s

# Searching Control
PEEK_DURATION = 1.5  # seconds
PEEK_ANGULAR_SPEED = 0.4  # rad/s
SWEEP_DURATION = 3.0  # seconds
SWEEP_ANGULAR_SPEED = 0.6  # rad/s

# Obstacle Avoidance Control
OBSTACLE_TURN_ANGULAR_SPEED = 1.0  # rad/s
SIDE_OBSTACLE_ADJUST_ANGULAR_SPEED = 0.7  # rad/s
CRITICAL_BACKWARD_SPEED = -0.1  # m/s
MANEUVER_FORWARD_SPEED = 0.1 # m/s
MANEUVER_BACKWARD_SPEED = -0.1 # m/s

# --- Color Tracking Constants ---
# HSV Color Range for mask (e.g., for a blue shirt)
# These values are examples. You might need to tune them.
# Or, better, extract them dynamically from the target.
LOWER_HUE = 100
UPPER_HUE = 140

class QRCodeFollower(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        # ROS Communications
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.user_profile_pub = self.create_publisher(String, USER_PROFILE_TOPIC, 10)
        self.image_sub = self.create_subscription(CompressedImage, IMAGE_TOPIC, self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_callback, 10)

        # YOLO Model (for initial detection)
        self.model = YOLO(YOLO_MODEL_PATH)
        os.makedirs(USER_DATA_DIR, exist_ok=True)

        # --- State Management ---
        self.robot_state = 'IDLE'  # IDLE, FOLLOWING, SEARCHING
        self.locked_target_id = None
        
        # --- Tracking Info ---
        self.tracking_mode = 'YOLO' # YOLO or COLOR
        self.target_hist = None     # Histogram for color tracking
        self.track_window = None    # Tracking window for MeanShift/CamShift

        # Obstacle Avoidance
        self.lidar_zones = {'front': float('inf'), 'left': float('inf'), 'right': float('inf')}

        # Searching State
        self.search_state = 'PEEK'
        self.search_start_time = 0
        self.search_state_start_time = 0
        self.last_seen_side = 'CENTER'
        self.sweep_direction = 1

    def scan_callback(self, msg):
        # Wider LiDAR zones for better obstacle detection
        self.lidar_zones['right'] = min([r for r in msg.ranges[270:330] if r > 0.01] or [float('inf')])
        self.lidar_zones['front'] = min([r for r in (msg.ranges[330:360] + msg.ranges[0:30]) if r > 0.01] or [float('inf')])
        self.lidar_zones['left']  = min([r for r in msg.ranges[30:90] if r > 0.01] or [float('inf')])

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        frame_height, frame_width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        final_twist = Twist()
        
        if self.robot_state == 'IDLE':
            self.tracking_mode = 'YOLO' # Reset to YOLO mode when idle
            self.target_hist = None
            self.track_window = None
            
            qr_codes = decode(frame)
            if qr_codes:
                # --- Use YOLO only when a QR code is detected to find the person ---
                results = self.model(frame, verbose=False)[0]
                person_boxes = [list(map(int, box.xyxy[0])) for box in results.boxes if self.model.names[int(box.cls[0])] == 'person']

                for qr in qr_codes:
                    try:
                        user_info = json.loads(qr.data.decode("utf-8"))
                        user_id = user_info.get('user_id')
                        x, y, w, h = qr.rect
                        qr_center_x, qr_center_y = x + w // 2, y + h // 2
                        
                        for p_box in person_boxes:
                            if p_box[0] < qr_center_x < p_box[2] and p_box[1] < qr_center_y < p_box[3]:
                                self.get_logger().info(f'New target locked: {user_id}. Initializing color tracking.')
                                self.locked_target_id = user_id
                                
                                # --- Initialize Color Tracking ---
                                self.initialize_color_tracker(frame, hsv, p_box)
                                
                                self.robot_state = 'FOLLOWING'
                                self.tracking_mode = 'COLOR'
                                self.save_user_profile(user_info)
                                self.user_profile_pub.publish(String(data=json.dumps(user_info)))
                                break # Exit person_boxes loop
                    except Exception as e:
                        self.get_logger().error(f'QR parsing or YOLO matching failed: {e}')
                    if self.robot_state == 'FOLLOWING':
                        break # Exit qr_codes loop
        
        elif self.robot_state == 'FOLLOWING':
            if self.tracking_mode == 'COLOR' and self.target_hist is not None:
                # Calculate back projection
                dst = cv2.calcBackProject([hsv], [0], self.target_hist, [0, 180], 1)
                
                # Apply meanShift to get the new location
                ret, self.track_window = cv2.meanShift(dst, self.track_window, (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1))

                if ret > 0: # If tracking is successful
                    px, py, pw, ph = self.track_window
                    final_twist = self.generate_motion_command((px, py, px+pw, py+ph), frame_width)
                    # Visualization
                    cv2.rectangle(frame, (px, py), (px + pw, py + ph), (0, 255, 0), 3)
                    cv2.putText(frame, f"Tracking: {self.locked_target_id}", (px, py - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else: # Tracking failed
                    self.get_logger().warn("Color tracking failed. Switching to SEARCHING.")
                    self.robot_state = 'SEARCHING'
                    self.search_start_time = time.time()
                    self.search_state_start_time = time.time()
            else:
                # Fallback or error
                self.get_logger().error("In FOLLOWING state but no tracking mode or histogram. Returning to IDLE.")
                self.robot_state = 'IDLE'

        elif self.robot_state == 'SEARCHING':
            # Try to find QR code again to re-lock
            qr_codes = decode(frame)
            reacquired = False
            for qr in qr_codes:
                try:
                    user_info = json.loads(qr.data.decode("utf-8"))
                    if user_info.get('user_id') == self.locked_target_id:
                        self.get_logger().info("Re-acquired target via QR code!")
                        self.robot_state = 'IDLE' # Go to IDLE to re-initiate tracking
                        reacquired = True
                        break
                except:
                    pass
            
            if not reacquired:
                final_twist = self.get_searching_twist()
                if self.robot_state != 'SEARCHING': # If search timed out
                    self.target_hist = None # Clear tracking data
                    self.track_window = None

        self.cmd_vel_pub.publish(final_twist)
        cv2.imshow("Hybrid Tracker", frame)
        cv2.waitKey(1)

    def initialize_color_tracker(self, frame, hsv, person_box):
        x, y, w, h = person_box[0], person_box[1], person_box[2]-person_box[0], person_box[3]-person_box[1]
        
        # --- Define Region of Interest (ROI) for color sampling ---
        # Focus on the center of the upper body to get shirt color
        roi_x = x + w // 4
        roi_y = y + h // 6 # Start a bit below the top to avoid head
        roi_w = w // 2
        roi_h = h // 2
        self.track_window = (roi_x, roi_y, roi_w, roi_h)
        
        # Get the ROI for calculating the histogram
        roi = hsv[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        
        # Create a mask to exclude dark/bright/desaturated pixels
        mask = cv2.inRange(roi, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
        
        # Calculate histogram of the ROI
        self.target_hist = cv2.calcHist([roi], [0], mask, [180], [0, 180])
        cv2.normalize(self.target_hist, self.target_hist, 0, 255, cv2.NORM_MINMAX)
        
        # For visualization of the ROI
        cv2.rectangle(frame, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), (255, 0, 0), 2)


    def generate_motion_command(self, target_box, frame_width):
        # This function now receives the tracked window (x,y,w,h) or a person_box (x1,y1,x2,y2)
        # It needs to be robust to both formats.
        if len(target_box) == 4:
             # Check if it's (x,y,w,h) or (x1,y1,x2,y2)
            if target_box[2] > frame_width or target_box[3] > frame_height: # Heuristic: if x2/y2 are larger than frame, it's a box
                x1, y1, x2, y2 = target_box
                w = x2 - x1
                h = y2 - y1
            else: # It's a window (x,y,w,h)
                x1, y1 = target_box[0], target_box[1]
                w, h = target_box[2], target_box[3]
        else:
            self.get_logger().error("Invalid target_box format in generate_motion_command")
            return Twist()

        # 0. Critical Obstacle Avoidance
        if (self.lidar_zones['front'] < CRITICAL_SAFETY_DISTANCE or
            self.lidar_zones['left'] < CRITICAL_SAFETY_DISTANCE or
            self.lidar_zones['right'] < CRITICAL_SAFETY_DISTANCE):
            self.get_logger().warn("CRITICAL OBSTACLE! Moving backward.")
            twist_cmd = Twist()
            twist_cmd.linear.x = CRITICAL_BACKWARD_SPEED
            twist_cmd.angular.z = 0.0
            return twist_cmd

        # 1. Front Obstacle Avoidance
        if self.lidar_zones['front'] < SAFETY_DISTANCE:
            self.get_logger().warn("Obstacle FRONT! Turning.")
            twist_cmd = Twist()
            twist_cmd.linear.x = 0.0
            twist_cmd.angular.z = -OBSTACLE_TURN_ANGULAR_SPEED if self.lidar_zones['left'] < self.lidar_zones['right'] else OBSTACLE_TURN_ANGULAR_SPEED
            return twist_cmd

        # 2. State-Based Action (Following)
        base_twist = self.get_following_twist((x1, y1, w, h), frame_width)

        # 3. Side Obstacle Avoidance
        side_obstacle_detected = False
        if self.lidar_zones['left'] < SAFETY_DISTANCE:
            self.get_logger().warn("Obstacle LEFT! Adjusting motion to turn right.")
            base_twist.angular.z -= SIDE_OBSTACLE_ADJUST_ANGULAR_SPEED
            side_obstacle_detected = True
        if self.lidar_zones['right'] < SAFETY_DISTANCE:
            self.get_logger().warn("Obstacle RIGHT! Adjusting motion to turn left.")
            base_twist.angular.z += SIDE_OBSTACLE_ADJUST_ANGULAR_SPEED
            side_obstacle_detected = True

        if side_obstacle_detected:
            if base_twist.linear.x > 0:
                base_twist.linear.x = min(base_twist.linear.x, MANEUVER_FORWARD_SPEED)
            elif base_twist.linear.x < 0:
                base_twist.linear.x = max(base_twist.linear.x, MANEUVER_BACKWARD_SPEED)
        
        return base_twist

    def get_following_twist(self, target_window, frame_width):
        # target_window is (x, y, w, h)
        twist_msg = Twist()
        x, y, w, h = target_window
        target_center_x = x + w // 2
        target_area = w * h
        
        error_x = target_center_x - frame_width // 2
        twist_msg.angular.z = ANGULAR_KP * error_x
        
        if target_area < MIN_TARGET_AREA:
            twist_msg.linear.x = FORWARD_SPEED
        elif target_area > MAX_TARGET_AREA:
            twist_msg.linear.x = BACKWARD_SPEED
        else:
            twist_msg.linear.x = 0.0
            
        # Update last seen side for searching
        if target_center_x < frame_width / 3: self.last_seen_side = 'LEFT'
        elif target_center_x > frame_width * 2 / 3: self.last_seen_side = 'RIGHT'
        else: self.last_seen_side = 'CENTER'
            
        return twist_msg

    def get_searching_twist(self):
        if time.time() - self.search_start_time > LOST_TIMEOUT:
            self.get_logger().info("Search timeout. Returning to IDLE state.")
            self.robot_state = 'IDLE'
            self.locked_target_id = None
            return Twist()

        twist_msg = Twist()
        current_time = time.time()
        if self.search_state == 'PEEK':
            if current_time - self.search_state_start_time > PEEK_DURATION:
                self.search_state = 'SWEEP'
                self.search_state_start_time = current_time
            else:
                twist_msg.angular.z = -PEEK_ANGULAR_SPEED if self.last_seen_side == 'RIGHT' else PEEK_ANGULAR_SPEED
        elif self.search_state == 'SWEEP':
            if current_time - self.search_state_start_time > SWEEP_DURATION:
                self.sweep_direction *= -1
                self.search_state_start_time = current_time
            twist_msg.angular.z = SWEEP_ANGULAR_SPEED * self.sweep_direction
        return twist_msg

    def save_user_profile(self, user_data):
        user_id = user_data.get("user_id", "unknown")
        with open(os.path.join(USER_DATA_DIR, f"{user_id}.json"), "w") as f:
            json.dump(user_data, f, indent=4)
        self.get_logger().info(f'User profile saved: {USER_DATA_DIR}/{user_id}.json')

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()