#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RobotSync(Node):
    def __init__(self):
        super().__init__('robot_sync')
       
        # QoS 프로필 설정 (센서 데이터용)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # ← 이게 핵심!
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
       
        # 실제 로봇 데이터 구독 (QoS 적용)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.sync_position, sensor_qos)
       
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)
       
        # 명령 토픽은 기본 QoS 사용
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
       
        print("🔄 실제 로봇 ↔ Gazebo 동기화 시작! (QoS 수정됨)")

    def sync_position(self, msg):
        """실제 로봇 위치 출력"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        print(f"🤖 실제 로봇 위치: x={x:.2f}, y={y:.2f}")

    def scan_callback(self, msg):
        """LiDAR 데이터 처리"""
        min_dist = min([r for r in msg.ranges if r > 0.01] or [float('inf')])
        print(f"🔍 최근접 장애물: {min_dist:.2f}m")

    def cmd_callback(self, msg):
        """제어 명령 확인"""
        print(f"🎮 제어 명령: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")

def main():
    rclpy.init()
    node = RobotSync()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
