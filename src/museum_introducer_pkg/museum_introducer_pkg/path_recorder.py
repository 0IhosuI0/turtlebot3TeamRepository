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

        # /odom 구독
        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 1초마다 경로 저장
        self.timer = self.create_timer(1, self.record_position)

    def recommend_exhibit(self):
        exhibit_name = "💡 추천 전시품: 미래관 2번 부스 – 로봇팔 전시"
        self.get_logger().info(exhibit_name)

    def odom_callback(self, msg):
        self.latest_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def record_position(self):
        if hasattr(self, 'latest_pose'):
            self.path.append(self.latest_pose)
            self.get_logger().info(f"기록 중... 현재 위치: {self.latest_pose}")

    def destroy_node(self):
        super().destroy_node()

        # 경로 저장
        save_path = os.path.expanduser('~/turtlebor3TeamRepository/map/path/saved_path.txt')
        with open(save_path, 'w') as f:
            for pos in self.path:
                f.write(f"{pos[0]:.2f}, {pos[1]:.2f}\n")
        print(f"\n📍 이동 경로가 {save_path} 에 저장되었습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()