
# 스마트 TV 생산 라인 자동화 솔루션

## 프로젝트 개요

스마트 TV 생산 라인의 효율성을 극대화하기 위해 설계된 자동화 솔루션입니다.

Intel RealSense D435i 카메라, YOLOv5 객체 탐지 모델, Dobot Magician 로봇 암, 컨베이어 벨트 제어 시스템을 사용하여 생산 과정을 모니터링하고 자동화 작업을 수행합니다.

ROS2 기반 서버는 데이터를 관리하며 RoboDK 시뮬레이션과 통합된 동작을 실행합니다.

---

## 시연 영상
[스마트 TV 생산 라인 자동화 시연 영상](https://www.youtube.com/watch?v=IgKFjTNAdM4)

---

## 개발 환경

- **운영 체제**: Ubuntu 20.04 (서버), Raspberry Pi OS (컨베이어 제어)
- **프로그래밍 언어**: Python 3.8
- **주요 하드웨어**:
  - Dobot Magician 로봇
  - Intel RealSense D435i 카메라
  - Raspberry Pi 컨베이어 제어
- **주요 소프트웨어**:
  - YOLOv5 객체 탐지 모델
  - ROS2 (로봇 제어 및 데이터 통신)
  - RoboDK (시뮬레이션)

---

## 전체 시스템 흐름

1. **서버 연결**: 
    - WebSocket으로 ROS2 서버와 클라이언트를 연결.
    - 명령 송수신 및 상태 데이터 관리.

2. **객체 탐지**:
    - Intel RealSense 카메라로 패널 이미지를 실시간 스트리밍.
    - YOLOv5로 패널 탐지 및 분류.

3. **패널 분류 및 작업**:
    - 탐지된 패널 정보를 기반으로 컨베이어 벨트와 서보 모터를 제어하여 패널 분류.

4. **RoboDK 시뮬레이션과 동기화**:
    - 작업 순서를 시뮬레이션에 반영.
    - 패널 유형에 따라 시뮬레이션 작업을 실행.

5. **작업 완료 후 리셋**:
    - 다음 작업을 위한 초기화.

---

## 서버 기능

### 1. WebSocket 통신
- **기능 이름**: WebSocket 서버 연결
- **기능 설명**: 서버와 클라이언트를 WebSocket으로 연결하여 명령 송수신 및 상태 관리.

```python
import asyncio

async def websocket_server():
    """WebSocket 서버로 클라이언트 연결 관리"""
    server = await asyncio.start_server(handle_client, '0.0.0.0', 12345)
    print("WebSocket 서버가 12345 포트에서 실행 중입니다.")
    async with server:
        await server.serve_forever()

async def handle_client(reader, writer):
    """클라이언트와의 데이터 송수신"""
    while True:
        try:
            data = await reader.read(1024)
            if not data:
                print("클라이언트 연결 종료.")
                break
            command = data.decode().strip()
            print(f"수신된 명령: {command}")
            response = f"Received: {command}"
            writer.write(response.encode())
            await writer.drain()
        except Exception as e:
            print(f"오류 발생: {e}")
            break
    writer.close()
    await writer.wait_closed()

# WebSocket 서버 실행
asyncio.run(websocket_server())
```

---

### 2. YOLOv5 객체 탐지
- **기능 이름**: 패널 객체 탐지
- **기능 설명**: Intel RealSense D435i 카메라로 캡처한 이미지를 기반으로 YOLOv5로 패널 탐지.

```python
import cv2
import torch

# YOLOv5 모델 로드
model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
camera = cv2.VideoCapture(0)

def detect_panel():
    """YOLOv5를 이용한 패널 탐지"""
    ret, frame = camera.read()
    if ret:
        results = model(frame)
        for result in results.xyxy[0]:  # 탐지된 객체
            class_id = int(result[5])
            if class_id == 0:
                return "board"  # 보드 패널
            elif class_id == 1:
                return "back"  # 백 패널
    return None

# 카메라 종료
camera.release()
```

---

## 컨베이어 벨트 기능

### 1. GPIO 제어
- **기능 이름**: 컨베이어 벨트 동작
- **기능 설명**: Raspberry Pi GPIO 핀을 이용해 컨베이어 벨트를 동작.

```python
import RPi.GPIO as GPIO
import time

LEFT_BELT = 17
RIGHT_BELT = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_BELT, GPIO.OUT)
GPIO.setup(RIGHT_BELT, GPIO.OUT)

def move_left():
    """컨베이어 벨트 좌측 이동"""
    GPIO.output(LEFT_BELT, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(LEFT_BELT, GPIO.LOW)

def move_right():
    """컨베이어 벨트 우측 이동"""
    GPIO.output(RIGHT_BELT, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(RIGHT_BELT, GPIO.LOW)

# GPIO 종료
GPIO.cleanup()
```

---

## 로봇 암 기능

### 1. Dobot Magician 제어
- **기능 이름**: 로봇 암으로 패널 옮기기
- **기능 설명**: Dobot Magician을 사용해 보드 패널과 백 패널을 컨베이어로 옮김.

```python
from pydobot import Dobot

port = '/dev/ttyUSB0'
device = Dobot(port)

def move_board_panel():
    """보드 패널을 컨베이어로 이동"""
    device.move_to(250, 0, 50, 0)
    device.suck(True)
    device.move_to(250, 0, -50, 0)
    device.move_to(150, 100, 50, 0)
    device.suck(False)

def move_back_panel():
    """백 패널을 컨베이어로 이동"""
    device.move_to(300, 50, 50, 0)
    device.suck(True)
    device.move_to(300, 50, -50, 0)
    device.move_to(200, -100, 50, 0)
    device.suck(False)

device.close()
```

---

## 시뮬레이션 기능

### 1. RoboDK 제어
- **기능 이름**: RoboDK 시뮬레이션 작업 실행
- **기능 설명**: 패널 종류에 따라 RoboDK 시뮬레이션에서 작업 수행.

```python
from robodk import robolink

RDK = robolink.Robolink()

def simulate_board_panel():
    """보드 패널 작업 시뮬레이션 실행"""
    program = RDK.Item('Board_Panel_Task', robolink.ITEM_TYPE_PROGRAM)
    program.RunProgram()

def simulate_back_panel():
    """백 패널 작업 시뮬레이션 실행"""
    program = RDK.Item('Back_Panel_Task', robolink.ITEM_TYPE_PROGRAM)
    program.RunProgram()
```

