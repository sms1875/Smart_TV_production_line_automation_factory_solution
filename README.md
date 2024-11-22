# Smart_TV_production_line_automation_factory_solution

## 소개
스마트 TV 생산 라인의 효율성을 극대화하기 위한 자동화 솔루션입니다.
Intel RealSense D435i 카메라와 YOLOv5 객체 탐지 모델을 활용하여 생산 과정의 상태를 모니터링하고 Dobot Magician 로봇 암과 컨베이어 벨트를 ROS2 기반으로 제어합니다.

**Youtube 시연 영상**: https://www.youtube.com/watch?v=IgKFjTNAdM4

## 개발 환경
- 통신 : WebSocket을 통한 커맨드 전달
  
--- 

### 서버
- 운영 체제: Ubuntu + ROS2
- 하드웨어: Intel RealSense D435i
- 소프트웨어: YOLOv5, WebSocket
  
### 컨베이어 벨트
- 제어 장치: Raspberry Pi
  
### 로봇 암
- 모델: Dobot Magician
- 제어 환경: ROS2
  
### 시뮬레이션
- 플랫폼: RoboDK

--- 

## 기능 설명

1. 서버 연결 및 데이터 관리
    - WebSocket을 통해 ROS 서버와 클라이언트를 연결하고 명령을 송수신.
    - 서버 연결 상태를 실시간으로 확인하며 다중 클라이언트 연결을 처리.

2. 객체 탐지 및 명령 전송
    - YOLOv5로 객체 탐지 및 ROI(Region of Interest) 내 탐지된 객체 처리.
    - 객체 감지 상태에 따라 컨베이어 벨트 및 서보 모터 명령 실행:
        - Back Panel: 서보 모터 좌측 이동
        - Board Panel: 서보 모터 우측 이동
        - 미탐지 상태: 모터 정지
3. 카메라 데이터 스트리밍 및 이미지 처리
    - Intel RealSense D435i 카메라로 컬러 이미지 수집.
    - 탐지 결과를 ROS2 메시지로 퍼블리시하여 시각화.
4. 컨베이어 벨트 및 서보 모터 제어
    - 컨베이어 벨트:
        - 가속 및 감속 제어.
    - 방향 변경 및 정지 상태 제어.
    - 서보 모터:
        - 각도 조절 (최대 180도 회전) 및 초기화.

---


## 코드 주요 특징
### 서버 측 (Python 기반)
1. YOLOv5 및 Intel RealSense 사용
    - YOLOv5 객체 탐지 모델로 ROI 내 탐지.
    - RealSense 카메라로 실시간 프레임 수집.
2. WebSocket 통신
    - TCP 기반 WebSocket으로 클라이언트 명령 수신 및 결과 반환.
3. ROS2 통합
    - ROS2 메시지로 탐지 결과를 퍼블리시하여 상태 시각화.

### 컨베이어 벨트 및 모터 제어 코드 특징
1. GPIO 관리
    - Raspberry Pi GPIO를 활용한 핀 제어.
    - 스텝 모터의 속도 및 방향 제어.
2. 가속 및 감속 처리
    - 초기 속도에서 목표 속도로의 점진적 가속 구현.
3. 서보 모터 제어
    - 펄스 폭 변조(PWM)를 이용해 서보 모터 회전 각도 제어.
4. 서버 통신
    - 컨베이어 동작 상태를 서버 명령에 따라 업데이트.

## 전체 시스템 흐름

1. 서버 연결:
    - Raspberry Pi와 ROS 서버가 WebSocket으로 통신.
    - 클라이언트 명령을 수신하여 처리.

2. 객체 탐지:
    - Intel RealSense 카메라로 수집된 영상에서 YOLOv5를 이용해 객체 탐지.
    - 탐지 결과를 바탕으로 컨베이어 벨트 및 서보 모터 명령 실행.
3. 컨베이어 제어:
    - Raspberry Pi에서 GPIO로 컨베이어 및 서보 모터 제어.

4. 실시간 데이터 시각화:
    - ROS2 메시지를 통해 탐지 결과를 이미지로 시각화.
