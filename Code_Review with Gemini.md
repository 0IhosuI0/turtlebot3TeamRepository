# `qr_code_follower.py` 코드 리뷰 (Gemini 작성)

이 문서는 `qr_code_follower.py` 파일의 각 모듈과 함수들을 단계별로 설명합니다.

---

### `qr_code_follower.py` 파일 구조 및 주요 모듈/함수 설명

**1. `QRCodeFollower` 클래스 (메인 모듈)**

이 클래스는 ROS 노드로서, 로봇의 동작을 제어하는 모든 로직을 포함합니다.

*   **`__init__(self)` 함수:**
    *   **역할:** `QRCodeFollower` 노드를 초기화하고 필요한 통신 채널, 모델, 상태 변수 등을 설정합니다.
    *   **세부 설명:**
        *   `super().__init__('qr_code_follower')`: ROS 노드를 `qr_code_follower`라는 이름으로 초기화합니다.
        *   `self.bridge = CvBridge()`: OpenCV 이미지와 ROS 이미지 메시지 간의 변환을 위한 `CvBridge` 객체를 생성합니다.
        *   **ROS 통신 설정:**
            *   `self.cmd_vel_pub`: 로봇의 속도 명령(`Twist` 메시지)을 `/cmd_vel` 토픽으로 발행하는 퍼블리셔를 생성합니다.
            *   `self.user_profile_pub`: 인식된 사용자 프로필(`String` 메시지)을 `user_profile` 토픽으로 발행하는 퍼블리셔를 생성합니다.
            *   `self.image_sub`: 카메라 이미지(`CompressedImage` 메시지)를 `camera/image_raw/compressed` 토픽에서 구독하는 서브스크립션을 생성하고, `self.image_callback` 함수를 콜백으로 연결합니다.
            *   `self.scan_sub`: LiDAR 스캔 데이터(`LaserScan` 메시지)를 `/scan` 토픽에서 구독하는 서브스크립션을 생성하고, `self.scan_callback` 함수를 콜백으로 연결합니다.
        *   **YOLO 모델 로드:** `self.model = YOLO('yolov8n.pt')`를 사용하여 YOLOv8n 모델을 로드합니다. 이 모델은 이미지에서 사람을 감지하는 데 사용됩니다.
        *   `os.makedirs("users", exist_ok=True)`: 사용자 프로필을 저장할 `users` 디렉토리가 없으면 생성합니다.
        *   **상태 관리 변수 초기화:**
            *   `self.robot_state`: 로봇의 현재 상태를 나타냅니다 (`IDLE`, `FOLLOWING`, `SEARCHING`). 초기값은 `IDLE`입니다.
            *   `self.locked_target_id`: 현재 로봇이 추적하고 있는 대상의 ID를 저장합니다.
            *   `self.last_seen_box`: 마지막으로 감지된 대상의 바운딩 박스 정보를 저장합니다.
        *   **장애물 회피 변수 초기화:**
            *   `self.safety_distance`: 장애물 회피를 위한 안전 거리를 설정합니다 (0.5m).
            *   `self.lidar_zones`: LiDAR 데이터를 기반으로 전방, 좌측, 우측의 장애물까지의 거리를 저장하는 딕셔너리입니다.
        *   **탐색 상태 변수 초기화:** 로봇이 대상을 잃었을 때 탐색 동작을 제어하기 위한 변수들입니다 (`search_state`, `search_start_time`, `last_seen_side`, `sweep_direction`, `lost_timeout`).

*   **`scan_callback(self, msg)` 함수:**
    *   **역할:** LiDAR 스캔 데이터를 처리하여 로봇 주변의 장애물 정보를 업데이트합니다.
    *   **세부 설명:**
        *   `msg.ranges`에서 LiDAR 거리 데이터를 가져옵니다.
        *   `self.lidar_zones` 딕셔너리에 전방(330-360도 및 0-30도), 좌측(30-90도), 우측(270-330도) 영역의 최소 거리를 계산하여 저장합니다. 0.01보다 큰 유효한 거리 값만 고려합니다.

*   **`image_callback(self, msg)` 함수:**
    *   **역할:** 카메라 이미지 데이터를 처리하고, 사람 감지, QR 코드 인식, 대상 추적, 상태 전환 및 로봇 동작 명령 생성을 수행하는 핵심 함수입니다.
    *   **세부 설명:**
        *   **이미지 변환:** `CvBridge`를 사용하여 ROS `CompressedImage` 메시지를 OpenCV 이미지(`frame`)로 변환합니다.
        *   **YOLO를 이용한 사람 감지:** `self.model(frame, verbose=False)[0]`를 사용하여 이미지에서 사람을 감지하고, 감지된 사람들의 바운딩 박스 정보를 `person_boxes` 리스트에 저장합니다.
        *   **대상 감지 및 상태 업데이트:**
            *   **대상 재획득 (Locked 상태일 때):** `self.locked_target_id`가 설정되어 있으면, 이전에 본 대상의 위치(`self.last_seen_box`)를 기반으로 현재 프레임에서 가장 가까운 사람을 찾아 대상을 재획득하려 시도합니다.
            *   **새로운 대상 찾기 (IDLE 상태일 때):** `self.locked_target_id`가 없으면, 이미지에서 QR 코드를 디코딩합니다.
                *   디코딩된 QR 코드 데이터가 유효한 JSON 형식이고 `user_id`를 포함하면, 해당 QR 코드의 중심이 사람의 바운딩 박스 안에 있는지 확인합니다.
                *   조건이 만족되면, 해당 사람을 새로운 대상으로 `locked_target_id`로 설정하고, 로봇 상태를 `FOLLOWING`으로 변경합니다.
                *   `self.save_user_profile` 함수를 호출하여 사용자 프로필을 저장하고, `user_profile_pub`을 통해 사용자 프로필을 발행합니다.
        *   **상태 전환:**
            *   대상이 현재 프레임에서 보이면 로봇 상태를 `FOLLOWING`으로 설정하고, 마지막으로 본 대상의 바운딩 박스와 위치(`last_seen_side`)를 업데이트합니다.
            *   대상이 보이지 않고 로봇 상태가 `FOLLOWING`이었다면, 대상을 잃었다고 판단하고 로봇 상태를 `SEARCHING`으로 변경하며 탐색 타이머를 시작합니다.
        *   **동작 명령 생성 및 발행:**
            *   `self.generate_motion_command` 함수를 호출하여 현재 상태와 대상 정보를 기반으로 로봇의 속도 명령(`Twist` 메시지)을 생성합니다.
            *   생성된 속도 명령을 `self.cmd_vel_pub`을 통해 `/cmd_vel` 토픽으로 발행합니다.
        *   **시각화:** 감지된 대상의 바운딩 박스와 ID를 이미지에 표시하고, `cv2.imshow`를 통해 화면에 출력합니다.

*   **`generate_motion_command(self, target_box, frame_width)` 함수:**
    *   **역할:** 로봇의 최종 동작 명령(`Twist` 메시지)을 생성하는 함수로, 장애물 회피, 상태 기반 동작, 측면 장애물 회피의 세 가지 계층으로 구성됩니다.
    *   **세부 설명:**
        *   **1. 장애물 회피 계층:**
            *   `self.lidar_zones['front']` (전방 거리)가 `self.safety_distance`보다 작으면, 즉 전방에 장애물이 있으면 로봇을 정지시키고, 좌우 LiDAR 거리 중 더 먼 쪽으로 회전하여 장애물을 회피합니다.
        *   **2. 상태 기반 동작 계층:**
            *   로봇의 `robot_state`에 따라 다른 동작을 수행합니다.
                *   `FOLLOWING` 상태: `self.get_following_twist` 함수를 호출하여 대상을 따라가는 속도 명령을 생성합니다.
                *   `SEARCHING` 상태: `self.get_searching_twist` 함수를 호출하여 대상을 탐색하는 속도 명령을 생성합니다.
                *   `IDLE` 상태: 로봇을 정지시킵니다.
        *   **3. 반응형 측면 장애물 회피 계층:**
            *   `self.lidar_zones['left']` 또는 `self.lidar_zones['right']`가 `self.safety_distance`보다 작으면, 즉 측면에 장애물이 있으면 로봇의 전진 속도를 줄이고 장애물 반대 방향으로 회전하는 각속도를 추가하여 회피합니다.

*   **`get_following_twist(self, target_box, frame_width)` 함수:**
    *   **역할:** `FOLLOWING` 상태일 때 대상을 따라가는 속도 명령을 계산합니다.
    *   **세부 설명:**
        *   대상 바운딩 박스(`target_box`)의 중심 `x` 좌표와 이미지의 중심 `x` 좌표 간의 오차(`error_x`)를 계산합니다.
        *   `error_x`에 비례하여 각속도(`angular.z`)를 설정하여 대상을 이미지 중앙에 오도록 로봇을 회전시킵니다.
        *   대상 바운딩 박스의 면적(`target_area`)을 계산하여 대상과의 거리를 추정합니다.
            *   면적이 작으면 (대상이 멀리 있으면) 전진 속도(`linear.x`)를 양수로 설정하여 다가갑니다.
            *   면적이 크면 (대상이 가까이 있으면) 전진 속도를 음수로 설정하여 뒤로 물러납니다.
            *   적절한 면적 범위 내에 있으면 전진 속도를 0으로 설정하여 정지합니다.

*   **`get_searching_twist(self)` 함수:**
    *   **역할:** `SEARCHING` 상태일 때 대상을 탐색하는 속도 명령을 계산합니다.
    *   **세부 설명:**
        *   **탐색 시간 초과:** `self.lost_timeout` (15초) 동안 대상을 찾지 못하면 `IDLE` 상태로 돌아가고 `locked_target_id`를 초기화합니다.
        *   **탐색 상태 (`search_state`):**
            *   `PEEK` 상태: 대상을 잃은 방향(`self.last_seen_side`)으로 짧게 회전하여 대상을 다시 찾으려 시도합니다 (1.5초).
            *   `SWEEP` 상태: `PEEK` 상태에서 대상을 찾지 못하면, 좌우로 번갈아 가며 넓게 회전하여 대상을 탐색합니다 (3.0초마다 방향 전환).

*   **`save_user_profile(self, user_data)` 함수:**
    *   **역할:** 인식된 사용자 프로필 데이터를 JSON 파일로 저장합니다.
    *   **세부 설명:**
        *   `user_data` 딕셔너리에서 `user_id`를 가져와 파일 이름으로 사용합니다.
        *   `users/{user_id}.json` 경로에 사용자 데이터를 JSON 형식으로 저장합니다.

---

**2. `main(args=None)` 함수:**

*   **역할:** ROS 시스템을 초기화하고 `QRCodeFollower` 노드를 실행합니다.
*   **세부 설명:**
    *   `rclpy.init(args=args)`: ROS 2 클라이언트 라이브러리를 초기화합니다.
    *   `node = QRCodeFollower()`: `QRCodeFollower` 클래스의 인스턴스를 생성하여 노드를 만듭니다.
    *   `rclpy.spin(node)`: 노드의 콜백 함수들이 실행되도록 스핀합니다. 이 함수는 노드가 종료될 때까지 블록됩니다.
    *   `node.destroy_node()`: 노드가 종료될 때 노드와 관련된 리소스를 해제합니다.
    *   `rclpy.shutdown()`: ROS 2 클라이언트 라이브러리를 종료합니다.

---

**3. `if __name__ == '__main__':` 블록:**

*   **역할:** 이 스크립트가 직접 실행될 때 `main()` 함수를 호출하도록 합니다.
*   **세부 설명:** Python 스크립트의 표준 진입점입니다. 이 파일이 직접 실행될 경우 `main()` 함수가 호출되어 ROS 노드가 시작됩니다.

---

요약하자면, `qr_code_follower.py`는 `QRCodeFollower` 클래스를 중심으로 카메라와 LiDAR 센서 데이터를 활용하여 사람을 감지하고, QR 코드를 통해 특정 대상을 식별하여 추적하며, 동시에 장애물을 회피하는 로봇의 자율 주행 로직을 구현하고 있습니다.
