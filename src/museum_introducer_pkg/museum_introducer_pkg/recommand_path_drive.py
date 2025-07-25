import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


class PathFollower(Node):
    def __init__(self):
        super().__init__('recommand_path_drive')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/museum/navigation_goal', self.goal_callback, 10)

        self.target_pose = None
        self.current_pose = None
        self.timer = self.create_timer(0.1, self.follow_goal)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.target_pose = msg.pose

    def follow_goal(self):
        if self.current_pose is None or self.target_pose is None:
            return

        target_x = self.target_pose.position.x
        target_y = self.target_pose.position.y
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - yaw)

        twist = Twist()

        # 회전 우선
        if abs(angle_diff) > 0.1:
            twist.angular.z = 0.4 * angle_diff
        elif distance > 0.2:
            twist.linear.x = 0.3
        else:
            self.get_logger().info(f"목표 도착: ({target_x:.2f}, {target_y:.2f})")
            self.target_pose = None # 목표 도착 시 초기화

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
