import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class PathFollower(Node):
    def __init__(self):
        super().__init__('recommand_path_drive')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 좌표 리스트 (사진 기준)
        self.path = [
            (-7.95, 6.09),
            (7.17, 6.09),
            (7.17, -1.78),
            (7.17, -5.56),
            (7.17, 6.09),
            (-7.95, 6.09)
        ]

        self.index = 0
        self.pose = None
        self.timer = self.create_timer(0.1, self.follow_path)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def follow_path(self):
        if self.pose is None or self.index >= len(self.path):
            return

        target_x, target_y = self.path[self.index]
        current_x = self.pose.position.x
        current_y = self.pose.position.y

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        yaw = self.get_yaw_from_quaternion(self.pose.orientation)
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - yaw)

        twist = Twist()

        # 회전 우선
        if abs(angle_diff) > 0.1:
            twist.angular.z = 0.4 * angle_diff
        elif distance > 0.2:
            twist.linear.x = 0.3
        else:
            self.get_logger().info(f"도착: {self.index+1}/{len(self.path)} → ({target_x:.2f}, {target_y:.2f})")
            self.index += 1

        self.cmd_pub.publish(twist)

    def get_yaw_from_quaternion(self, q):
        # 오일러 각도 추출
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # -pi ~ pi 범위로 변환
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
