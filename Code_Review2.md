### ROS를 처음 접하는 분들을 위한 기본 개념

이 코드를 이해하기 전에, 로봇 소프트웨어 개발에 널리 사용되는 **ROS (Robot Operating System)**에 대한 몇 가지 기본 개념을 알아두면 좋습니다.

*   **노드 (Node)**: ROS에서 실행되는 가장 작은 프로그램 단위입니다. 예를 들어, 카메라에서 이미지를 받아오는 노드, 로봇을 움직이는 명령을 보내는 노드 등이 있습니다. 이 코드에서는 `qr_code_follower`라는 노드를 만듭니다.
*   **토픽 (Topic)**: 노드들이 서로 정보를 주고받는 통신 채널입니다. 특정 토픽에 정보를 '발행(publish)'하는 노드가 있고, 그 토픽의 정보를 '구독(subscribe)'하는 노드가 있습니다.
*   **메시지 (Message)**: 토픽을 통해 주고받는 실제 데이터 형식입니다. 이미지 데이터, 로봇 속도 명령 등 다양한 종류의 메시지가 있습니다.
*   **퍼블리셔 (Publisher)**: 특정 토픽에 메시지를 발행하는 역할을 합니다.
*   **서브스크라이버 (Subscriber)**: 특정 토픽의 메시지를 구독하여 받아오는 역할을 합니다.

---

### `qr_code_follower.py` 코드 설명 (초보자용)

이 파이썬 코드는 로봇이 카메라를 사용하여 QR 코드를 찾고, 그 QR 코드를 계속 따라가도록 만드는 프로그램입니다. 마치 로봇이 QR 코드를 '눈으로 보고' '따라가는' 행동을 하는 것과 같습니다.

#### 1. 필요한 도구들 불러오기 (모듈 임포트)

```python
import rclpy                 # ROS 2 통신을 위한 기본 도구
from rclpy.node import Node  # ROS 2 노드를 만들기 위한 기본 틀
from sensor_msgs.msg import Image # 카메라 이미지 데이터를 담는 메시지 형식
from geometry_msgs.msg import Twist # 로봇의 움직임(속도) 명령을 담는 메시지 형식
from cv_bridge import CvBridge # ROS 이미지와 OpenCV 이미지 변환 도구
import cv2                   # 이미지 처리(OpenCV) 도구
import numpy as np           # 숫자 계산 도구 (이 코드에서는 직접 사용은 적음)
from pyzbar import pyzbar    # QR 코드 읽기 도구
```

*   **`rclpy`**: 로봇이 ROS라는 언어로 다른 프로그램들과 대화할 수 있게 해주는 가장 기본적인 도구입니다.
*   **`rclpy.node.Node`**: 우리가 만들 `qr_code_follower`라는 프로그램(노드)이 ROS 시스템의 일부가 될 수 있도록 해주는 '설계도' 같은 것입니다.
*   **`sensor_msgs.msg.Image`**: 로봇의 카메라가 찍은 사진(이미지) 데이터를 ROS 시스템 안에서 주고받을 때 사용하는 '표준 형식'입니다.
*   **`geometry_msgs.msg.Twist`**: 로봇에게 "앞으로 가라", "왼쪽으로 돌아라" 같은 움직임 명령을 내릴 때 사용하는 '표준 형식'입니다. `linear.x`는 앞뒤 속도, `angular.z`는 회전 속도를 나타냅니다.
*   **`cv_bridge.CvBridge`**: 로봇 카메라에서 오는 이미지 데이터는 ROS만의 특별한 형식으로 되어 있습니다. 하지만 우리가 이미지 처리에 사용할 `cv2` (OpenCV)는 다른 형식을 사용합니다. `CvBridge`는 이 두 형식 사이를 번역해주는 '통역사' 역할을 합니다.
*   **`cv2`**: 'OpenCV'라고 불리는 강력한 이미지 처리 라이브러리입니다. 이미지에서 선을 그리거나, 특정 모양을 찾거나, 색깔을 바꾸는 등의 작업을 할 수 있게 해줍니다.
*   **`numpy`**: 파이썬에서 복잡한 숫자 계산을 효율적으로 할 수 있게 해주는 도구입니다. 이미지 데이터도 결국 숫자의 배열이기 때문에, 이미지 처리와 관련하여 많이 사용됩니다.
*   **`pyzbar`**: 이미지 안에서 QR 코드나 바코드를 찾아내고, 그 안에 담긴 정보를 읽어내는 전문 도구입니다.

#### 2. `QRCodeFollower` 로봇 두뇌 클래스 (Node)

이 `QRCodeFollower`라는 클래스는 로봇이 QR 코드를 따라가는 모든 행동을 결정하고 실행하는 '두뇌' 역할을 하는 프로그램(노드)입니다.

##### 2.1. `__init__` 초기 설정 (생성자)

```python
class QRCodeFollower(Node):
    def __init__(self):
        super().__init__('qr_code_follower') # 이 노드의 이름을 'qr_code_follower'로 정합니다.
        self.bridge = CvBridge() # 이미지 변환 통역사를 준비합니다.

        # 카메라 이미지 받기 (구독자 설정)
        # '/camera/image_raw' 토픽에서 Image 메시지를 받으면 image_callback 함수를 실행합니다.
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # 로봇 움직임 명령 보내기 (발행자 설정)
        # '/cmd_vel' 토픽으로 Twist 메시지를 보낼 준비를 합니다.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_x = 320  # QR 코드가 이미지의 이 X 좌표에 오도록 로봇을 움직일 것입니다. (가운데)
        self.kp = 0.002      # QR 코드가 중앙에서 벗어난 정도에 따라 로봇이 얼마나 빨리 돌아야 할지 결정하는 값 (비례 상수)
        self.linear_speed = 0.1 # 로봇이 앞으로 움직일 일정한 속도
```

*   `super().__init__('qr_code_follower')`: 이 프로그램(노드)의 이름을 `qr_code_follower`라고 정하고, ROS 시스템에 "나 이런 이름으로 활동할 거야!"라고 알리는 부분입니다.
*   `self.bridge = CvBridge()`: 위에서 설명한 이미지 통역사를 사용할 준비를 합니다.
*   `self.image_sub = self.create_subscription(...)`: 로봇의 카메라가 찍은 사진을 받아오기 위해 '구독자'를 만듭니다.
    *   `Image`: 받아올 데이터가 이미지 형식이라는 뜻입니다.
    *   `/camera/image_raw`: 카메라가 이미지를 보내는 '토픽'의 이름입니다.
    *   `self.image_callback`: 이 토픽으로 새로운 이미지가 도착할 때마다 `image_callback`이라는 함수를 실행하라고 지시합니다.
    *   `10`: 메시지를 처리할 때 잠시 저장해두는 공간(큐)의 크기입니다.
*   `self.cmd_vel_pub = self.create_publisher(...)`: 로봇에게 움직임 명령을 보내기 위해 '발행자'를 만듭니다.
    *   `Twist`: 보낼 데이터가 로봇의 속도 명령 형식이라는 뜻입니다.
    *   `/cmd_vel`: 로봇이 움직임 명령을 받아들이는 '토픽'의 이름입니다.
*   `self.target_x = 320`: 로봇의 카메라 화면이 보통 가로 640픽셀인데, 그 중앙은 320픽셀입니다. 로봇은 QR 코드를 이 320픽셀 위치에 오도록 계속 조절할 것입니다.
*   `self.kp = 0.002`: QR 코드가 `target_x`에서 얼마나 벗어났는지(오차)에 따라 로봇이 얼마나 빨리 회전해야 할지를 결정하는 '조절 값'입니다. 이 값이 클수록 로봇은 더 민감하게 반응합니다.
*   `self.linear_speed = 0.1`: 로봇이 QR 코드를 따라갈 때, 앞으로 움직이는 속도를 0.1로 고정합니다.

##### 2.2. `image_callback` 이미지 처리 및 로봇 제어 함수

이 함수는 카메라에서 새로운 이미지가 들어올 때마다 자동으로 실행됩니다. 로봇의 '눈' 역할을 하는 부분입니다.

```python
    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV가 이해할 수 있는 이미지로 변환합니다.
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            # 이미지 변환 중 문제가 생기면 오류 메시지를 출력하고 함수를 종료합니다.
            self.get_logger().error(f"Error converting image: {e}")
            return

        # 이미지에서 QR 코드를 찾습니다.
        decoded_objects = pyzbar.decode(cv_image)
        qr_code_found = False # QR 코드를 찾았는지 여부를 기록하는 변수

        for obj in decoded_objects: # 찾은 모든 객체(바코드, QR코드 등)를 하나씩 살펴봅니다.
            if obj.type == 'QRCODE': # 만약 현재 객체가 QR 코드라면
                qr_code_found = True # QR 코드를 찾았다고 표시합니다.
                points = obj.polygon # QR 코드의 네 모서리 점들을 가져옵니다.

                if len(points) == 4: # 네 모서리 점이 정확히 4개라면 (정상적인 QR 코드라면)
                    # QR 코드의 가장 왼쪽, 오른쪽, 위쪽, 아래쪽 좌표를 찾습니다.
                    x_coords = [p.x for p in points]
                    y_coords = [p.y for p in points]
                    x_min, x_max = min(x_coords), max(x_coords)
                    y_min, y_max = min(y_coords), max(y_coords)

                    # 이미지에 QR 코드 주변에 초록색 사각형을 그립니다.
                    cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                    # QR 코드의 정확한 중심점 X, Y 좌표를 계산합니다.
                    qr_center_x = int((x_min + x_max) / 2)
                    qr_center_y = int((y_min + y_max) / 2)

                    # 이미지에 QR 코드 중심에 빨간색 점을 그립니다.
                    cv2.circle(cv_image, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)

                    # QR 코드의 중심 X 좌표가 우리가 원하는 목표 X 좌표(target_x)에서 얼마나 벗어났는지 계산합니다.
                    # 이 값이 0이면 QR 코드가 화면 중앙에 있다는 뜻입니다.
                    error_x = self.target_x - qr_center_x

                    # 오차(error_x)에 비례 상수(kp)를 곱하여 로봇이 얼마나 회전해야 할지 결정합니다.
                    # 오차가 크면 크게 돌고, 작으면 작게 돕니다.
                    angular_z = self.kp * error_x

                    # 로봇에게 보낼 움직임 명령(Twist 메시지)을 만듭니다.
                    twist_msg = Twist()
                    twist_msg.linear.x = self.linear_speed # 앞으로 일정한 속도로 움직입니다.
                    twist_msg.angular.z = angular_z       # 계산된 회전 속도로 돕니다.
                    self.cmd_vel_pub.publish(twist_msg) # 로봇에게 이 명령을 보냅니다.

                    # 현재 상태를 화면에 출력합니다. (디버깅용)
                    self.get_logger().info(f"QR Code detected. Center X: {qr_center_x}, Error X: {error_x}, Angular Z: {angular_z}")
                break # 여러 QR 코드가 있어도 첫 번째 찾은 것만 처리하고 다음 이미지로 넘어갑니다.

        if not qr_code_found: # 만약 이미지에서 QR 코드를 전혀 찾지 못했다면
            # 로봇을 멈추는 명령을 보냅니다.
            twist_msg = Twist()
            twist_msg.linear.x = 0.0 # 앞으로 가는 속도 0
            twist_msg.angular.z = 0.0 # 회전 속도 0
            self.cmd_vel_pub.publish(twist_msg) # 로봇에게 멈추라고 명령합니다.
            self.get_logger().info("No QR Code detected. Stopping robot.") # 멈췄다고 출력합니다.

        # 로봇이 보고 있는 화면을 컴퓨터 화면에 보여줍니다.
        cv2.imshow("QR Code Follower", cv_image)
        cv2.waitKey(1) # 화면이 잘 보이도록 잠시 기다립니다.
```

*   **이미지 변환**: 카메라에서 받은 ROS 형식의 이미지를 `cv_bridge`를 이용해 `cv2` (OpenCV)가 처리할 수 있는 일반적인 이미지 형식으로 바꿉니다.
*   **QR 코드 찾기**: `pyzbar.decode()` 함수를 사용하여 이미지 안에서 QR 코드를 찾아냅니다. 찾은 QR 코드의 위치와 정보를 알아냅니다.
*   **QR 코드 위치 계산**: 찾은 QR 코드의 네 모서리 점을 이용하여 QR 코드의 정확한 중심점(X, Y 좌표)을 계산합니다.
*   **오차 계산**: QR 코드의 중심 X 좌표가 우리가 원하는 화면 중앙(`self.target_x`)에서 얼마나 떨어져 있는지 '오차'를 계산합니다.
*   **로봇 회전 명령 계산**: 이 오차에 `self.kp` 값을 곱하여 로봇이 얼마나 빨리 회전해야 할지 결정합니다. QR 코드가 화면 중앙에서 오른쪽으로 벗어나면 로봇은 오른쪽으로 돌고, 왼쪽으로 벗어나면 왼쪽으로 돌게 됩니다.
*   **로봇 움직임 명령 보내기**: `Twist` 메시지를 만들어서 로봇에게 "앞으로 `self.linear_speed` 속도로 가면서, `angular_z` 속도로 돌아라"라고 명령을 보냅니다. 이 명령은 `/cmd_vel` 토픽을 통해 로봇에게 전달됩니다.
*   **QR 코드 못 찾았을 때**: 만약 카메라 화면에서 QR 코드를 찾지 못하면, 로봇이 헤매지 않도록 모든 속도를 0으로 설정하여 로봇을 멈춥니다.
*   **화면 표시**: 로봇이 현재 보고 있는 카메라 화면과, 그 위에 QR 코드를 찾아서 표시한 사각형/점을 컴퓨터 화면에 보여줍니다.

#### 3. `main` 함수: 프로그램 시작 및 종료 관리

```python
def main(args=None):
    rclpy.init(args=args) # ROS 시스템을 시작할 준비를 합니다.
    qr_code_follower = QRCodeFollower() # 위에서 만든 로봇 두뇌(노드)를 실제로 만듭니다.
    rclpy.spin(qr_code_follower) # 로봇 두뇌가 계속해서 카메라 이미지를 받고 명령을 보내도록 무한 반복시킵니다.
                                 # 이 부분이 없으면 프로그램이 바로 끝나버립니다.
    qr_code_follower.destroy_node() # 프로그램이 끝날 때, 로봇 두뇌를 깔끔하게 정리합니다.
    rclpy.shutdown() # ROS 시스템을 종료합니다.
```

*   `rclpy.init(args=args)`: ROS 시스템을 사용하기 위한 초기 설정을 합니다.
*   `qr_code_follower = QRCodeFollower()`: 우리가 만든 `QRCodeFollower` 클래스를 사용하여 실제 로봇 두뇌(노드)를 하나 만듭니다.
*   `rclpy.spin(qr_code_follower)`: 이 함수는 `qr_code_follower` 노드가 계속해서 작동하도록 무한히 반복시켜 줍니다. 카메라 이미지가 들어오면 `image_callback` 함수를 실행하고, 로봇 명령을 보내는 등의 모든 작업을 이 `spin` 함수가 관리합니다. 이 함수가 없으면 노드가 시작되자마자 바로 끝나버립니다.
*   `qr_code_follower.destroy_node()`: 프로그램이 종료될 때, 사용했던 자원들을 깨끗하게 정리합니다.
*   `rclpy.shutdown()`: ROS 시스템과의 연결을 완전히 끊고 종료합니다.

#### 4. 스크립트 실행 시작점

```python
if __name__ == '__main__':
    main() # 이 파이썬 파일을 직접 실행하면 main 함수를 호출합니다.
```

*   이 부분은 이 파이썬 파일을 직접 실행했을 때만 `main()` 함수를 호출하도록 하는 표준적인 파이썬 코드입니다. 다른 파이썬 파일에서 이 코드를 불러와 사용할 때는 `main()` 함수가 자동으로 실행되지 않습니다.

---

요약하자면, 이 코드는 로봇의 카메라로 들어오는 영상을 실시간으로 분석하여 QR 코드를 찾고, 그 QR 코드가 항상 화면 중앙에 오도록 로봇의 회전 속도를 조절하면서 앞으로 나아가게 하는 프로그램입니다. ROS라는 시스템을 통해 카메라 데이터와 로봇 제어 명령을 주고받습니다.

이 설명이 ROS를 처음 접하는 데 도움이 되기를 바랍니다. 추가적으로 궁금한 점이 있다면 언제든지 질문해주세요.