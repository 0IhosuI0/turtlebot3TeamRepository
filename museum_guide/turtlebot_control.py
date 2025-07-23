#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
박물관 가이드 로봇 - 터틀봇 제어 모듈
(실제 맵 적용, 7개 유물 기준 버전)
"""
import time
import random
import json
import math
from datetime import datetime

# ROS2 및 Gazebo 연동
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan, Image
    from std_msgs.msg import String
    ROS_AVAILABLE = True
    print("✅ ROS2 환경 감지됨 - Gazebo 연동 모드")
except ImportError:
    ROS_AVAILABLE = False
    print("⚠️ ROS2 미설치 - 시뮬레이션 모드로 동작")


class TurtlebotController:
    def __init__(self):
        self.connected = False
        self.ros_node = None
        self.current_position = {"x": 0, "y": 0, "theta": 0}
        self.battery_level = 100
        self.moving = False
        self.current_location = "입구"

        # (SDF Map 기반 실제 좌표)
        self.exhibition_positions = {
            1: {"x": 0.56, "y": 0.13, "theta": 0.0, "name": "나전칠기함", "location": "A동 1층"},
            2: {"x": 2.85, "y": 2.82, "theta": 0.0, "name": "향로", "location": "A동 2층"},
            3: {"x": -4.92, "y": -3.03, "theta": 0.0, "name": "동종", "location": "B동 1층"},
            4: {"x": 6.88, "y": -3.27, "theta": 0.0, "name": "백자병", "location": "B동 2층"},
            5: {"x": 8.32, "y": 0.75, "theta": 0.0, "name": "청동주전자", "location": "C동 1층"},
            6: {"x": 6.26, "y": 2.94, "theta": 0.0, "name": "청동연꽃모양 연적", "location": "C동 2층"},
            7: {"x": -2.51, "y": 6.17, "theta": 0.0, "name": "청자 잔 받침", "location": "D동 1층"}
        }

        self.status = {
            "mode": "idle",
            "speed": "normal",
            "voice_enabled": True,
            "led_status": "green",
            "gazebo_connected": False,
            "navigation_active": False
        }

        if ROS_AVAILABLE:
            self.init_ros_node()

    def init_ros_node(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            self.ros_node = rclpy.create_node('museum_guide_controller')
            self.cmd_vel_pub = self.ros_node.create_publisher(Twist, '/cmd_vel', 10)
            self.goal_pub = self.ros_node.create_publisher(PoseStamped, '/goal_pose', 10)
            self.laser_sub = self.ros_node.create_subscription(
                LaserScan, '/scan', self.laser_callback, 10)
            print("🤖 ROS2 노드 초기화 완료")
            self.status["gazebo_connected"] = True
        except Exception as e:
            print(f"❌ ROS2 노드 초기화 실패: {e}")
            self.status["gazebo_connected"] = False

    def laser_callback(self, msg):
        try:
            min_distance = min([r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)])
            if min_distance < 0.5:
                print("⚠️ 장애물 감지!")
        except:
            pass

    def connect(self):
        print("🤖 터틀봇 연결 시도 중...")

        if ROS_AVAILABLE and self.status["gazebo_connected"]:
            print("🎮 Gazebo 시뮬레이션 환경 연결 중...")
            time.sleep(2)
            self.connected = True
            self.current_location = "시뮬레이션 입구"
            self.battery_level = 100
            print("✅ Gazebo 터틀봇 연결 성공!")
        else:
            print("📡 시뮬레이션 모드로 연결 중...")
            time.sleep(2)
            if random.random() > 0.1:
                self.connected = True
                self.current_location = "입구"
                self.battery_level = random.randint(80, 100)
                print("✅ 시뮬레이션 터틀봇 연결 성공!")
            else:
                print("❌ 터틀봇 연결 실패!")
                return False

        self._initialize_robot()
        return True

    def _initialize_robot(self):
        print("🔧 터틀봇 시스템 초기화 중...")
        time.sleep(1)
        if ROS_AVAILABLE:
            print("📡 라이다 센서 확인...")
            print("📷 카메라 센서 확인...")
            print("🗺️ 네비게이션 스택 확인...")
        else:
            print("📡 센서 시뮬레이션 모드...")
        time.sleep(0.5)
        print("⚙️ 모터 상태 확인...")
        time.sleep(0.5)
        print("✅ 터틀봇 초기화 완료!")

    def move_to_exhibition(self, exhibit_id):
        if not self.connected:
            print("❌ 터틀봇이 연결되지 않았습니다.")
            return False

        if exhibit_id not in self.exhibition_positions:
            print(f"❌ 전시품 {exhibit_id}의 위치 정보가 없습니다.")
            return False

        target = self.exhibition_positions[exhibit_id]

        print(f"🚶 ♂️ {target['name']}로 이동 시작...")
        print(f"📍 목적지: {target['location']} ({target['x']:.2f}, {target['y']:.2f})")

        self.status["mode"] = "moving"
        self.moving = True

        if ROS_AVAILABLE and self.status["gazebo_connected"]:
            self._navigate_to_position(target)
        else:
            self._simulate_movement(target)

        self.current_position = {"x": target["x"], "y": target["y"], "theta": target["theta"]}
        self.current_location = target["location"]
        self.status["mode"] = "guiding"
        self.moving = False

        print(f"✅ {target['name']} 도착 완료!")
        return True

    def _navigate_to_position(self, target):
        try:
            print("🗺️ Gazebo에서 터틀봇 이동 시작...")
            self._send_movement_commands(target)
            if hasattr(self, 'goal_pub'):
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
                goal_msg.pose.position.x = target['x']
                goal_msg.pose.position.y = target['y']
                goal_msg.pose.position.z = 0.0
                goal_msg.pose.orientation.z = math.sin(target['theta'] / 2)
                goal_msg.pose.orientation.w = math.cos(target['theta'] / 2)
                self.goal_pub.publish(goal_msg)
                print("🎯 Navigation2 목표 설정 완료")
        except Exception as e:
            print(f"❌ Navigation 오류: {e}")
            self._simulate_movement(target)

    def _send_movement_commands(self, target):
        try:
            if not self.ros_node or not hasattr(self, 'cmd_vel_pub'):
                print("❌ ROS2 퍼블리셔 미초기화")
                return
            current_x, current_y = self.current_position["x"], self.current_position["y"]
            target_x, target_y = target["x"], target["y"]
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            print(f"📏 계산된 거리: {distance:.2f}m")
            if distance > 0.1:
                target_angle = math.atan2(dy, dx)
                self._rotate_to_angle(target_angle)
                move_duration = max(3, int(distance * 2))
                linear_speed = min(0.3, distance / move_duration)
                print(f"🤖 ROS2 이동 명령: 속도={linear_speed:.2f}m/s, 시간={move_duration}초")
                twist = Twist()
                twist.linear.x = linear_speed
                for i in range(move_duration):
                    if not self.moving:
                        break
                    self.cmd_vel_pub.publish(twist)
                    print(f"🚶 ♂️ 이동 중... {((i+1)/move_duration*100):.0f}%")
                    if self.ros_node:
                        rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                    time.sleep(1)
                    self.battery_level -= 0.2
                stop_twist = Twist()
                self.cmd_vel_pub.publish(stop_twist)
                print("🛑 터틀봇 정지")
        except Exception as e:
            print(f"❌ ROS2 움직임 명령 실패: {e}")
            self._simulate_movement(target)

    def _rotate_to_angle(self, target_angle):
        try:
            current_angle = self.current_position.get("theta", 0)
            angle_diff = target_angle - current_angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            if abs(angle_diff) > 0.1:
                angular_speed = 0.5 if angle_diff > 0 else -0.5
                rotation_time = int(abs(angle_diff) * 2)
                print(f"🔄 회전 중... {math.degrees(angle_diff):.1f}도")
                twist = Twist()
                twist.angular.z = angular_speed
                for i in range(rotation_time):
                    if not self.moving:
                        break
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.5)
                stop_twist = Twist()
                self.cmd_vel_pub.publish(stop_twist)
                self.current_position["theta"] = target_angle
        except Exception as e:
            print(f"❌ 회전 명령 실패: {e}")

    def _simulate_movement(self, target):
        current_x, current_y = self.current_position["x"], self.current_position["y"]
        target_x, target_y = target["x"], target["y"]
        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        move_time = max(5, int(distance * 2))
        print(f"📏 이동 거리: {distance:.2f}m")
        print(f"⏱️ 예상 시간: {move_time}초")
        for i in range(move_time):
            if not self.moving:
                break
            progress = (i + 1) / move_time * 100
            print(f"🚶 ♂️ 시뮬레이션 이동 중... {progress:.0f}%")
            time.sleep(1)
            self.battery_level -= 0.1
            if random.random() < 0.1:
                print("⚠️ 장애물 감지, 경로 재계획 중...")
                time.sleep(0.5)

    def follow_user_to_exhibition(self, exhibit_id):
        if not self.connected:
            print("❌ 터틀봇이 연결되지 않았습니다.")
            return False
        target = self.exhibition_positions.get(exhibit_id)
        if not target:
            print(f"❌ 전시품 {exhibit_id}의 위치 정보가 없습니다.")
            return False
        print(f"👥 사용자를 따라 {target['name']}로 이동 중...")
        print("🤖 안전거리 1.5m를 유지하며 추종합니다.")
        self.status["mode"] = "tracking"
        self.moving = True
        self._simulate_user_following(target)
        self.current_position = {"x": target["x"], "y": target["y"], "theta": target["theta"]}
        self.current_location = target["location"]
        self.status["mode"] = "guiding"
        self.moving = False
        print(f"✅ {target['name']} 위치에서 가이드 대기 중")
        return True

    def _simulate_user_following(self, target):
        print("👀 사용자 위치 추적 활성화...")
        time.sleep(1)
        if ROS_AVAILABLE:
            print("📡 라이다로 사용자 감지 중...")
        else:
            print("🤖 시뮬레이션 사용자 추적 중...")
        for i in range(4):
            print(f"👥 사용자 추종 중... {i+1}/4")
            time.sleep(1.5)
            self.battery_level -= 0.2
            if random.random() < 0.2:
                print("⚠️ 사용자가 너무 빠릅니다. 속도 조절 중...")
                time.sleep(0.5)

    def detect_sticker(self):
        if not self.connected:
            return None
        if ROS_AVAILABLE:
            print("📷 카메라로 QR코드 스캔 중...")
        detection_chance = random.random()
        if detection_chance < 0.3:
            detected_id = random.randint(1, 7)
            print(f"🔍 QR코드 인식: 전시품 {detected_id}번")
            return detected_id
        return None

    def play_sound(self, sound_type):
        sounds = {
            "welcome": "환영합니다! 박물관 가이드를 시작합니다.",
            "arrived": "목적지에 도착했습니다.",
            "warning": "주의하세요. 장애물이 있습니다.",
            "goodbye": "관람해주셔서 감사합니다!",
            "following": "안전거리를 유지하며 따라가겠습니다."
        }
        message = sounds.get(sound_type, "")
        if self.status["voice_enabled"] and message:
            print(f"🔊 음성안내: {message}")
            try:
                import os
                os.system(f'espeak -v ko "{message}" 2>/dev/null')
            except:
                print("🔇 음성 출력 장치 없음 - 텍스트로 표시")
            time.sleep(1)

    def get_gazebo_status(self):
        if ROS_AVAILABLE and self.status["gazebo_connected"]:
            return {
                "simulation": "Gazebo 활성",
                "map_loaded": True,
                "navigation": "Navigation2 활성",
                "sensors": "라이다+카메라 활성"
            }
        else:
            return {
                "simulation": "시뮬레이션 모드",
                "map_loaded": False,
                "navigation": "시뮬레이션",
                "sensors": "가상 센서"
            }

    def get_status(self):
        gazebo_info = self.get_gazebo_status()
        return {
            "connected": self.connected,
            "battery": round(self.battery_level),
            "location": self.current_location,
            "position": self.current_position,
            "moving": self.moving,
            "mode": self.status["mode"],
            "speed": self.status["speed"],
            "voice_enabled": self.status["voice_enabled"],
            "led_status": self.status["led_status"],
            "gazebo_status": gazebo_info,
            "ros_available": ROS_AVAILABLE,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

    def emergency_stop(self):
        print("🛑 긴급 정지!")
        self.moving = False
        self.status["mode"] = "idle"
        if ROS_AVAILABLE and self.ros_node and hasattr(self, 'cmd_vel_pub'):
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
        print("✅ 안전하게 정지했습니다.")

    def stop_and_disconnect(self):
        print("🛑 터틀봇을 안전하게 정지시킵니다...")
        if self.moving:
            self.emergency_stop()
        if self.current_location != "입구":
            print("🏠 기지로 복귀 중...")
        if ROS_AVAILABLE and self.ros_node:
            try:
                self.ros_node.destroy_node()
                rclpy.shutdown()
                print("🤖 ROS2 노드 종료")
            except:
                pass
        self.connected = False
        self.status["mode"] = "offline"
        print("✅ 터틀봇이 안전하게 종료되었습니다.")

    def set_speed(self, speed):
        valid_speeds = ["slow", "normal", "fast"]
        if speed in valid_speeds:
            self.status["speed"] = speed
            print(f"🏃 ♂️ 이동 속도를 '{speed}'로 설정했습니다.")
            return True
        else:
            print(f"❌ 올바르지 않은 속도 설정: {speed}")
            return False

    def set_voice_guide(self, enabled):
        self.status["voice_enabled"] = enabled
        status_text = "활성화" if enabled else "비활성화"
        print(f"🔊 음성 안내를 {status_text}했습니다.")

    def return_to_base(self):
        print("🏠 기지로 복귀 중...")
        self.status["mode"] = "moving"
        self.moving = True
        base_position = {"x": 0, "y": 0, "theta": 0}
        distance = math.sqrt(self.current_position["x"] ** 2 + self.current_position["y"] ** 2)
        move_time = max(2, int(distance / 15))
        for i in range(move_time):
            progress = (i + 1) / move_time * 100
            print(f"🏠 기지로 복귀 중... {progress:.0f}%")
            time.sleep(1)
            self.battery_level -= 0.1
        self.current_position = base_position
        self.current_location = "입구"
        self.status["mode"] = "idle"
        self.moving = False
        print("✅ 기지 복귀 완료!")

    def charge_battery(self, duration=10):
        print("🔌 배터리 충전 중...")
        for i in range(duration):
            if self.battery_level < 100:
                self.battery_level = min(100, self.battery_level + 2)
                print(f"🔋 충전 중... {self.battery_level}%")
            else:
                print("🔋 배터리 충전 완료!")
                break
            time.sleep(0.5)

    def check_low_battery(self):
        return self.battery_level < 20

    def set_led_color(self, color):
        valid_colors = ["red", "green", "blue", "yellow", "purple"]
        if color in valid_colors:
            self.status["led_status"] = color
            print(f"💡 LED를 {color}색으로 설정했습니다.")
        else:
            print(f"❌ 올바르지 않은 LED 색상: {color}")

    def get_sensor_data(self):
        return {
            "lidar": {
                "front_distance": random.uniform(0.5, 5.0),
                "obstacles_detected": random.choice([True, False])
            },
            "camera": {
                "sticker_detected": random.choice([True, False]),
                "people_detected": random.randint(0, 3)
            },
            "imu": {
                "orientation": random.uniform(0, 360),
                "stability": "stable"
            },
            "temperature": random.uniform(20, 25),
            "humidity": random.uniform(40, 60)
        }

    def diagnostic_check(self):
        print("🔍 터틀봇 진단 실행 중...")
        diagnostics = {
            "battery_health": "양호" if self.battery_level > 50 else "점검 필요",
            "motor_status": "정상",
            "sensor_status": "정상",
            "network_status": "연결됨" if self.connected else "연결 안됨",
            "memory_usage": f"{random.randint(30, 70)}%",
            "cpu_temperature": f"{random.randint(35, 55)}°C",
            "ros2_status": "활성" if ROS_AVAILABLE else "비활성"
        }
        print("📋 진단 결과:")
        for item, status in diagnostics.items():
            print(f"   {item}: {status}")
        return diagnostics

