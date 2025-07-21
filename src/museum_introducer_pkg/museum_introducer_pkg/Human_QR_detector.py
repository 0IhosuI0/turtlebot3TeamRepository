import cv2
import json
import os
from pyzbar.pyzbar import decode
from ultralytics import YOLO

# YOLO 모델 로드 (사람 감지)
model = YOLO('yolov8n.pt')  # yolov8n: 가장 가볍고 빠른 버전

# 저장 폴더 생성
os.makedirs("users", exist_ok=True)

def save_user_profile(user_data):
    user_id = user_data.get("user_id", "unknown")
    with open(f"users/{user_id}.json", "w") as f:
        json.dump(user_data, f, indent=4)
    print(f"[✔] 사용자 정보 저장됨: users/{user_id}.json")

def choose_central_person(boxes, frame_width):
    """프레임 중심에 가장 가까운 사람을 선택"""
    min_offset = float('inf')
    target_box = None
    for box in boxes:
        x1, y1, x2, y2 = box
        center_x = (x1 + x2) // 2
        offset = abs(center_x - frame_width // 2)
        if offset < min_offset:
            min_offset = offset
            target_box = box
    return target_box

# 카메라 열기 (터틀봇 카메라의 경우 /dev/video0 또는 ros 이미지 bridge 필요)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_height, frame_width = frame.shape[:2]

    # --- 사람 인식 (YOLO) ---
    results = model(frame, verbose=False)[0]
    person_boxes = []

    for box in results.boxes:
        cls_id = int(box.cls[0])
        if model.names[cls_id] == 'person':
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            person_boxes.append((x1, y1, x2, y2))
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # --- QR 코드 인식 ---
    qr_codes = decode(frame)
    for qr in qr_codes:
        qr_data = qr.data.decode("utf-8")
        try:
            user_info = json.loads(qr_data)
            save_user_profile(user_info)
            x, y, w, h = qr.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, f"{user_info['user_id']}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        except Exception as e:
            print(f"[!] QR 파싱 실패: {e}")

    # --- 타겟 사람 선택 표시 ---
    if person_boxes:
        target = choose_central_person(person_boxes, frame_width)
        if target:
            x1, y1, x2, y2 = target
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(frame, "Target", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # --- 프레임 출력 ---
    cv2.imshow("TurtleBot View (YOLO + QR)", frame)

    # 종료 키
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
