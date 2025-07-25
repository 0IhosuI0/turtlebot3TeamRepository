#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
박물관 가이드 로봇 - ROS2 패키지 연동 브리지
팀원들의 museum_introducer_pkg와 연동
"""

import json
import time
from datetime import datetime

# ROS2 연동을 위한 import
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Twist, PoseStamped
    from museum_robot_msgs.action import FollowUser
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    # print("⚠️ ROS2 미설치 - 시뮬레이션 모드로 동작") # Suppress this print

class MuseumROS2Bridge(Node):
    """팀원들의 ROS2 패키지와 우리 시스템을 연결하는 브리지"""
   
    def __init__(self):
        super().__init__('museum_guide_bridge')
        self.get_logger().info("MuseumROS2Bridge node initializing...")
       
        # Publishers - 팀원들 노드로 데이터 전송
        self.user_profile_pub = self.create_publisher(String, '/museum/user_profile', 10)
        self.exhibition_info_pub = self.create_publisher(String, '/museum/exhibition_info', 10)
        self.navigation_cmd_pub = self.create_publisher(PoseStamped, '/museum/navigation_goal', 10)

        # Action Client
        self._action_client = ActionClient(self, FollowUser, 'follow_user')
       
        # Subscribers - 팀원들 노드에서 데이터 수신
        self.qr_detection_sub = self.create_subscription(
            String, '/museum/qr_detected', self.qr_detection_callback, 10)
        self.human_detection_sub = self.create_subscription(
            String, '/museum/human_detected', self.human_detection_callback, 10)
        self.navigation_status_sub = self.create_subscription(
            String, '/museum/navigation_status', self.navigation_status_callback, 10)
       
        # 상태 변수
        self.detected_qr = None
        self.human_detected = False
        self.navigation_active = False
        self.robot_state = "IDLE"
       
        self.get_logger().info("ROS2 브리지 노드 초기화 완료")

    def qr_detection_callback(self, msg):
        try:
            qr_data = json.loads(msg.data)
            self.detected_qr = qr_data
            self.get_logger().debug(f"QR코드 감지: {qr_data}") # Changed to debug
           
            if 'exhibition_id' in qr_data:
                exhibition_id = qr_data['exhibition_id']
                self.get_logger().debug(f"전시품 {exhibition_id}번 인식됨") # Changed to debug
               
        except Exception as e:
            self.get_logger().error(f"QR코드 데이터 파싱 오류: {e}")

    def human_detection_callback(self, msg):
        try:
            human_data = json.loads(msg.data)
            self.human_detected = human_data.get('detected', False)
           
            if self.human_detected:
                self.get_logger().debug("사용자 감지됨") # Changed to debug
            else:
                self.get_logger().debug("사용자 감지 안됨") # Changed to debug
               
        except Exception as e:
            self.get_logger().error(f"사람 감지 데이터 파싱 오류: {e}")

    def navigation_status_callback(self, msg):
        try:
            nav_data = json.loads(msg.data)
            self.navigation_active = nav_data.get('active', False)
            status = nav_data.get('status', 'unknown')
           
            self.get_logger().debug(f"네비게이션 상태: {status}") # Changed to debug
           
        except Exception as e:
            self.get_logger().error(f"네비게이션 상태 파싱 오류: {e}")

    def publish_user_profile(self, user_profile):
        try:
            profile_data = {
                "user_id": user_profile.visitor_id,
                "nickname": user_profile.nickname,
                "age_group": user_profile.age_group,
                "interests": [user_profile.interest_field],
                "knowledge_level": user_profile.knowledge_level,
                "available_time": user_profile.available_time,
                "timestamp": datetime.now().isoformat()
            }
           
            msg = String()
            msg.data = json.dumps(profile_data, ensure_ascii=False)
            self.user_profile_pub.publish(msg)
           
            self.get_logger().debug(f"사용자 프로필 발행: {user_profile.nickname}") # Changed to debug
           
        except Exception as e:
            self.get_logger().error(f"사용자 프로필 발행 오류: {e}")

    def publish_exhibition_info(self, exhibition_id, exhibition_data):
        try:
            exhibition_info = {
                "exhibition_id": exhibition_id,
                "name": exhibition_data["name"],
                "location": exhibition_data["location"],
                "duration": exhibition_data["duration"],
                "position": exhibition_data.get("position", {}),
                "descriptions": exhibition_data.get("descriptions", {}),
                "timestamp": datetime.now().isoformat()
            }
           
            msg = String()
            msg.data = json.dumps(exhibition_info, ensure_ascii=False)
            self.exhibition_info_pub.publish(msg)
           
            self.get_logger().debug(f"전시품 정보 발행: {exhibition_data['name']}") # Changed to debug
           
        except Exception as e:
            self.get_logger().error(f"전시품 정보 발행 오류: {e}")

    def publish_navigation_goal(self, target_position):
        try:
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
           
            goal_msg.pose.position.x = target_position['x']
            goal_msg.pose.position.y = target_position['y']
            goal_msg.pose.position.z = 0.0
           
            goal_msg.pose.orientation.z = target_position.get('theta', 0.0)
            goal_msg.pose.orientation.w = 1.0
           
            self.navigation_cmd_pub.publish(goal_msg)
           
            self.get_logger().debug(f"네비게이션 목표 발행: ({target_position['x']}, {target_position['y']})") # Changed to debug
           
        except Exception as e:
            self.get_logger().error(f"네비게이션 목표 발행 오류: {e}")

    def send_follow_user_goal(self, user_id, command):
        goal_msg = FollowUser.Goal()
        goal_msg.user_id = user_id
        goal_msg.command = command

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.status}')
        self.robot_state = "IDLE"

    def feedback_callback(self, feedback_msg):
        self.robot_state = feedback_msg.feedback.current_state
        self.get_logger().debug(f'Received feedback: {self.robot_state}') # Changed to debug

    def get_detected_qr(self):
        detected = self.detected_qr
        self.detected_qr = None
        return detected

    def is_human_detected(self):
        return self.human_detected

    def is_navigation_active(self):
        return self.navigation_active

class ROS2IntegrationManager:
    def __init__(self, logger):
        self.logger = logger
        self.bridge = None
        self.ros_thread = None
       
        if ROS_AVAILABLE:
            try:
                # rclpy.init() # Moved to main.py
                self.bridge = MuseumROS2Bridge()
                self.logger.info("ROS2 브리지 초기화 성공")
            except Exception as e:
                self.logger.error(f"ROS2 브리지 초기화 실패: {e}")
                self.bridge = None

    def start_ros_spinning(self):
        if self.bridge:
            import threading
           
            def spin_ros():
                try:
                    rclpy.spin(self.bridge)
                except Exception as e:
                    self.get_logger().error(f"ROS2 스피닝 오류: {e}")
           
            self.ros_thread = threading.Thread(target=spin_ros, daemon=True)
            self.ros_thread.start()
            self.get_logger().info("ROS2 스피닝 시작")

    def stop_ros(self):
        if self.bridge:
            self.bridge.destroy_node()
            rclpy.shutdown()
            self.get_logger().info("ROS2 브리지 종료")

    def send_user_profile(self, user_profile):
        if self.bridge:
            self.bridge.publish_user_profile(user_profile)

    def send_exhibition_info(self, exhibition_id, exhibition_data):
        if self.bridge:
            self.bridge.publish_exhibition_info(exhibition_id, exhibition_data)

    def send_navigation_goal(self, target_position):
        if self.bridge:
            self.bridge.publish_navigation_goal(target_position)

    def start_following(self, user_id):
        if self.bridge:
            self.bridge.send_follow_user_goal(user_id, "START")

    def stop_following(self):
        if self.bridge:
            self.bridge.send_follow_user_goal("", "STOP")

    def get_robot_state(self):
        if self.bridge:
            return self.bridge.robot_state
        return "IDLE"

    def get_qr_detection(self):
        if self.bridge:
            return self.bridge.get_detected_qr()
        return None

    def check_human_detection(self):
        if self.bridge:
            return self.bridge.is_human_detected()
        return False

    def check_navigation_status(self):
        if self.bridge:
            return self.bridge.is_navigation_active()
        return False


# 팀원들의 사용자 데이터 로드 함수
def load_user_data_from_json(user_id="U128"):
    """팀원들이 만든 사용자 JSON 데이터 로드"""
    try:
        # Adjust path to be relative to the package's data directory
        script_dir = os.path.dirname(__file__)
        file_path = os.path.join(script_dir, '..', 'profiles', f'{user_id}.json')
        
        with open(file_path, 'r', encoding='utf-8') as f:
            user_data = json.load(f)
       
        print(f"✅ 사용자 데이터 로드: {user_data['name']}")
        return user_data
       
    except FileNotFoundError:
        print(f"❌ 사용자 파일 없음: {file_path}")
        return None
    except Exception as e:
        print(f"❌ 사용자 데이터 로드 오류: {e}")
        return None


def convert_user_interests_to_our_system(interests):
    """팀원들의 관심사를 우리 시스템 형식으로 변환"""
    interest_mapping = {
        "history": "역사",
        "sculpture": "예술",
        "art": "예술",
        "culture": "문화",
        "painting": "예술"
    }
   
    # 첫 번째 관심사를 기본으로 사용
    if interests and len(interests) > 0:
        first_interest = interests[0].lower()
        return interest_mapping.get(first_interest, "전체")
   
    return "전체"


# 테스트 함수
def test_integration():
    """통합 테스트"""
    print("🧪 ROS2 통합 테스트 시작")
   
    # ROS2 매니저 초기화
    rclpy.init()
    ros_manager = ROS2IntegrationManager()
    ros_manager.start_ros_spinning()
   
    # 사용자 데이터 로드 테스트
    user_data = load_user_data_from_json("U128")
    if user_data:
        print(f"👤 사용자: {user_data['name']}")
        print(f"🎨 관심사: {user_data['interests']}")
        converted_interest = convert_user_interests_to_our_system(user_data['interests'])
        print(f"🔄 변환된 관심사: {converted_interest}")
   
    # 몇 초 대기 후 종료
    time.sleep(5)
    ros_manager.stop_ros()
    print("✅ 통합 테스트 완료")


if __name__ == "__main__":
    test_integration()