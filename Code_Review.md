### 1. 모듈 임포트 (Imports)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar import pyzbar
```

*   **`rclpy`**: ROS 2 파이썬 클라이언트 라이브러리입니다. ROS 2 노드 생성 및 통신을 위한 기본 기능을 제공합니다.
*   **`rclpy.node.Node`**: ROS 2 노드를 생성하기 위한 기본 클래스입니다.
*   **`sensor_msgs.msg.Image`**: ROS 2에서 카메라 이미지 데이터를 전송하는 데 사용되는 메시지 타입입니다.
*   **`geometry_msgs.msg.Twist`**: 로봇의 선형 및 각속도를 제어하는 데 사용되는 메시지 타입입니다.
*   **`cv_bridge.CvBridge`**: ROS 이미지 메시지와 OpenCV 이미지 형식 간의 변환을 처리하는 라이브러리입니다.
*   **`cv2`**: OpenCV(Open Source Computer Vision Library) 파이썬 바인딩입니다. 이미지 처리 및 컴퓨터 비전 기능을 제공합니다.
*   **`numpy`**: 파이썬에서 수치 계산을 위한 라이브러리입니다. 이미지 데이터 처리 시 배열 연산에 사용될 수 있습니다. (이 코드에서는 직접적인 배열 연산보다는 `cv2` 내부에서 활용됩니다.)
*   **`pyzbar`**: 바코드 및 QR 코드를 디코딩하는 라이브러리입니다. 이미지에서 QR 코드를 찾아 그 정보를 추출하는 데 사용됩니다.

### 2. `QRCodeFollower` 클래스

이 클래스는 QR 코드 팔로잉 로직을 구현하는 ROS 2 노드입니다.

#### 2.1. `__init__` 메서드 (생성자)

```python
class QRCodeFollower(Node):
    def __init__(self):
        super().__init__('qr_code_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_x = 320  # Center of the image
        self.kp = 0.002      # Proportional gain for angular velocity
        self.linear_speed = 0.1 # Constant linear speed
```

*   `super().__init__('qr_code_follower')`: `Node` 클래스의 생성자를 호출하여 `qr_code_follower`라는 이름의 ROS 2 노드를 초기화합니다.
*   `self.bridge = CvBridge()`: `CvBridge` 객체를 생성하여 ROS 이미지와 OpenCV 이미지 간의 변환을 준비합니다.
*   `self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)`: `/camera/image_raw` 토픽에서 `Image` 메시지를 구독하는 서브스크라이버를 생성합니다. 메시지가 수신되면 `self.image_callback` 메서드가 호출됩니다. 큐 크기는 10입니다.
*   `self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)`: `/cmd_vel` 토픽에 `Twist` 메시지를 발행하는 퍼블리셔를 생성합니다. 로봇의 속도 명령을 전송하는 데 사용됩니다. 큐 크기는 10입니다.
*   `self.target_x = 320`: 이미지의 중앙 X 좌표를 나타내는 목표 값입니다. QR 코드가 이 X 좌표에 위치하도록 로봇이 회전합니다. (일반적인 640x480 이미지의 경우 중앙은 320입니다.)
*   `self.kp = 0.002`: 비례 제어(P-제어)를 위한 비례 이득(Proportional Gain) 값입니다. QR 코드의 X 좌표 오차에 이 값을 곱하여 로봇의 각속도를 결정합니다.
*   `self.linear_speed = 0.1`: 로봇이 QR 코드를 따라갈 때 유지할 일정한 선형 속도입니다.

#### 2.2. `image_callback` 메서드

```python
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        decoded_objects = pyzbar.decode(cv_image)
        qr_code_found = False
        for obj in decoded_objects:
            if obj.type == 'QRCODE':
                qr_code_found = True
                points = obj.polygon
                if len(points) == 4:
                    # Get bounding box coordinates
                    x_coords = [p.x for p in points]
                    y_coords = [p.y for p in points]
                    x_min, x_max = min(x_coords), max(x_coords)
                    y_min, y_max = min(y_coords), max(y_coords)

                    # Draw bounding box
                    cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                    # Calculate center of QR code
                    qr_center_x = int((x_min + x_max) / 2)
                    qr_center_y = int((y_min + y_max) / 2)

                    # Draw center point
                    cv2.circle(cv_image, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)

                    # Calculate error
                    error_x = self.target_x - qr_center_x

                    # Calculate angular velocity
                    angular_z = self.kp * error_x

                    # Publish Twist message
                    twist_msg = Twist()
                    twist_msg.linear.x = self.linear_speed
                    twist_msg.angular.z = angular_z
                    self.cmd_vel_pub.publish(twist_msg)
                    self.get_logger().info(f"QR Code detected. Center X: {qr_center_x}, Error X: {error_x}, Angular Z: {angular_z}")
                break # Process only the first QR code found

        if not qr_code_found:
            # Stop the robot if no QR code is found
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info("No QR Code detected. Stopping robot.")

        cv2.imshow("QR Code Follower", cv_image)
        cv2.waitKey(1)
```

*   **이미지 변환**:
    *   `cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')`: 수신된 ROS `Image` 메시지를 OpenCV에서 처리할 수 있는 `bgr8` 형식의 이미지(NumPy 배열)로 변환합니다.
    *   `try-except` 블록은 이미지 변환 중 발생할 수 있는 오류를 처리합니다.
*   **QR 코드 디코딩**:
    *   `decoded_objects = pyzbar.decode(cv_image)`: `pyzbar` 라이브러리를 사용하여 OpenCV 이미지에서 모든 바코드 및 QR 코드를 디코딩합니다.
    *   `qr_code_found = False`: QR 코드 발견 여부를 추적하는 플래그입니다.
*   **QR 코드 처리 루프**:
    *   `for obj in decoded_objects:`: 디코딩된 각 객체를 반복합니다.
    *   `if obj.type == 'QRCODE'`: 현재 객체가 QR 코드인지 확인합니다.
    *   `qr_code_found = True`: QR 코드가 발견되었음을 표시합니다.
    *   `points = obj.polygon`: QR 코드의 외곽선(폴리곤)을 구성하는 점들을 가져옵니다.
    *   `if len(points) == 4`: QR 코드가 사각형 형태(4개의 점)인지 확인합니다.
    *   **바운딩 박스 계산 및 그리기**:
        *   `x_coords`, `y_coords`: 폴리곤의 X, Y 좌표를 추출합니다.
        *   `x_min, x_max, y_min, y_max`: QR 코드의 최소/최대 X, Y 좌표를 찾아 바운딩 박스를 정의합니다.
        *   `cv2.rectangle(...)`: OpenCV를 사용하여 원본 이미지에 QR 코드의 바운딩 박스를 녹색으로 그립니다.
    *   **QR 코드 중심 계산 및 그리기**:
        *   `qr_center_x`, `qr_center_y`: 바운딩 박스의 중앙 X, Y 좌표를 계산합니다.
        *   `cv2.circle(...)`: OpenCV를 사용하여 QR 코드의 중심을 빨간색 원으로 그립니다.
    *   **오차 계산**:
        *   `error_x = self.target_x - qr_center_x`: QR 코드의 현재 X 좌표와 목표 X 좌표(`self.target_x`) 간의 오차를 계산합니다. 이 오차는 로봇이 얼마나 회전해야 하는지를 결정하는 데 사용됩니다.
    *   **각속도 계산**:
        *   `angular_z = self.kp * error_x`: 계산된 오차에 비례 이득(`self.kp`)을 곱하여 로봇의 각속도(`angular_z`)를 결정합니다. 오차가 클수록 더 빠르게 회전합니다.
    *   **`Twist` 메시지 발행**:
        *   `twist_msg = Twist()`: 새로운 `Twist` 메시지 객체를 생성합니다.
        *   `twist_msg.linear.x = self.linear_speed`: 로봇의 선형 속도를 `self.linear_speed`로 설정합니다.
        *   `twist_msg.angular.z = angular_z`: 로봇의 각속도를 계산된 `angular_z`로 설정합니다.
        *   `self.cmd_vel_pub.publish(twist_msg)`: 구성된 `Twist` 메시지를 `/cmd_vel` 토픽에 발행하여 로봇을 제어합니다.
        *   `self.get_logger().info(...)`: 현재 QR 코드 감지 상태 및 제어 정보를 로깅합니다.
    *   `break`: 여러 QR 코드가 감지될 경우 첫 번째 QR 코드만 처리하고 루프를 종료합니다.
*   **QR 코드 미발견 시 처리**:
    *   `if not qr_code_found:`: 이미지에서 QR 코드가 전혀 발견되지 않은 경우입니다.
    *   `twist_msg = Twist()`: 새로운 `Twist` 메시지를 생성합니다.
    *   `twist_msg.linear.x = 0.0`, `twist_msg.angular.z = 0.0`: 로봇의 선형 및 각속도를 모두 0으로 설정하여 로봇을 정지시킵니다.
    *   `self.cmd_vel_pub.publish(twist_msg)`: 정지 명령을 발행합니다.
    *   `self.get_logger().info(...)`: QR 코드가 감지되지 않아 로봇이 정지했음을 로깅합니다.
*   **이미지 표시**:
    *   `cv2.imshow("QR Code Follower", cv_image)`: 처리된 이미지를 "QR Code Follower"라는 창에 표시합니다.
    *   `cv2.waitKey(1)`: 1밀리초 동안 키 입력을 기다립니다. 이는 OpenCV 창이 올바르게 업데이트되도록 합니다.

### 3. `main` 함수

```python
def main(args=None):
    rclpy.init(args=args)
    qr_code_follower = QRCodeFollower()
    rclpy.spin(qr_code_follower)
    qr_code_follower.destroy_node()
    rclpy.shutdown()
```

*   `rclpy.init(args=args)`: ROS 2 시스템을 초기화합니다.
*   `qr_code_follower = QRCodeFollower()`: `QRCodeFollower` 노드 인스턴스를 생성합니다.
*   `rclpy.spin(qr_code_follower)`: 노드를 스핀(spin)합니다. 이는 노드의 콜백 함수(예: `image_callback`)가 호출되도록 이벤트를 처리하는 무한 루프를 시작합니다. 프로그램이 종료되거나 `rclpy.shutdown()`이 호출될 때까지 실행됩니다.
*   `qr_code_follower.destroy_node()`: 노드가 종료될 때 리소스를 해제합니다.
*   `rclpy.shutdown()`: ROS 2 시스템을 종료합니다.

### 4. 스크립트 실행 진입점

```python
if __name__ == '__main__':
    main()
```

*   이 블록은 스크립트가 직접 실행될 때 `main()` 함수를 호출하도록 합니다. 모듈로 임포트될 때는 실행되지 않습니다.

이 코드는 카메라에서 이미지를 받아 QR 코드를 감지하고, QR 코드의 위치에 따라 로봇의 회전 속도를 조절하여 QR 코드를 화면 중앙에 유지하며 따라가도록 설계되었습니다.
