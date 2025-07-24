import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        # 추천 전시품 텍스트 출력
        self.recommend_exhibit()

        # 경로 저장용 리스트
        self.path = []

        # 최신 위치 초기화
        self.latest_pose = None

        # /odom 구독
        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 1초마다 경로 저장
        self.timer = self.create_timer(1.0, self.record_position)

    def recommend_exhibit(self):
        exhibit_name = "💡 추천 전시품: 미래관 2번 부스 – 로봇팔 전시"
        self.get_logger().info(exhibit_name)

    def odom_callback(self, msg):
        self.latest_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def record_position(self):
        if self.latest_pose is not None:
            self.path.append(self.latest_pose)
            self.get_logger().info(f"기록 중... 현재 위치: {self.latest_pose}")

    def save_path_to_file(self):
        # 기본 디렉토리 설정
        base_dir = os.path.expanduser('~/turtlebot3TeamRepository/map/path')
        os.makedirs(base_dir, exist_ok=True)

        # 저장할 파일 이름에 번호 자동 추가
        i = 1
        while True:
            save_path = os.path.join(base_dir, f"saved_path_{i}.txt")
            if not os.path.exists(save_path):
                break
            i += 1

        # 파일에 경로 저장
        with open(save_path, 'w') as f:
            for pos in self.path:
                f.write(f"{pos[0]:.2f}, {pos[1]:.2f}\n")
        self.get_logger().info(f"\n📍 이동 경로가 {save_path} 에 저장되었습니다.")


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 종료 요청 받음 (Ctrl+C)')
    finally:
        node.save_path_to_file()
        node.destroy_node()
        rclpy.shutdown()
