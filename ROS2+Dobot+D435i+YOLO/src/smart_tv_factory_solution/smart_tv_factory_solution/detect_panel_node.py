import socket
import threading
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)
# FutureWarning: `torch.cuda.amp.autocast(args...)` is deprecated. Please use `torch.amp.autocast('cuda', args...)` instead.

class Config:
    """상수 및 설정"""
    ROI_POSITION = (181, 99, 490, 252)  # ROI 좌표
    SOCKET_PORT = 12345  # 소켓 서버 포트
    SOCKET_HOST = '0.0.0.0'  # 모든 인터페이스에서 접속 허용
    CAMERA_RESOLUTION = (640, 480)  # 카메라 해상도
    CAMERA_FPS = 30  # 카메라 FPS

class DetectPanelNode(Node):
    def __init__(self):
        super().__init__('detect_panel_node')

        # 초기화 호출
        self.yolo_model = None
        self.pipeline = None
        self.server_socket = None
        self.client_socket = None
        self.bridge = None

        self.init_model()
        self.init_camera()
        self.init_socket()
        self.init_ros()

    # ------------------------------
    # 초기화 메서드
    # ------------------------------
    def init_model(self):
        """YOLOv5 모델 초기화"""
        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
        self.yolo_model.eval()

    def init_camera(self):
        """RealSense 카메라 초기화"""
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, *Config.CAMERA_RESOLUTION, rs.format.bgr8, Config.CAMERA_FPS)
        self.pipeline.start(config)

    def init_socket(self):
        """소켓 서버 초기화"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((Config.SOCKET_HOST, Config.SOCKET_PORT))
        self.server_socket.listen(1)
        self.get_logger().info(f'Socket server listening on port {Config.SOCKET_PORT}')
        self.client_thread = threading.Thread(target=self.accept_client)
        self.client_thread.start()

    def init_ros(self):
        """ROS2 노드 초기화"""
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    # ------------------------------
    # 클라이언트 관리
    # ------------------------------
    def accept_client(self):
        """소켓 클라이언트 연결 대기"""
        self.client_socket, addr = self.server_socket.accept()
        self.get_logger().info(f'Client connected from {addr}')

    def send_results(self, detection_results):
        """탐지 결과를 클라이언트에 전송"""
        if self.client_socket:
            try:
                result_data = '\n'.join([f'{label} {color}' for label, color, _, _, _, _ in detection_results])
                self.client_socket.sendall(result_data.encode('utf-8') + b'\n')
            except BrokenPipeError:
                self.get_logger().info('Client disconnected.')
                self.client_socket = None

    # ------------------------------
    # 프레임 처리
    # ------------------------------
    def get_color_name(self, hsv_color):
        """HSV 색상 값에서 이름 반환"""
        h, s, v = hsv_color

        # White: 높은 V와 낮은 S
        if s < 50 and v > 200:
            return 'white'

        # Red: H가 0-10 또는 160-180 (빨간색은 양 끝에 위치)
        if (0 <= h <= 10 or 160 <= h <= 180) and s > 100 and v > 100:
            return 'red'

        # Blue: H가 100-140 (파란색 범위)
        if 100 <= h <= 140 and s > 100 and v > 100:
            return 'blue'

        return 'unknown'


    def get_center_color(self, image):
        """이미지 중심 영역의 평균 색상 반환"""
        height, width = image.shape[:2]
        center_y, center_x = height // 2, width // 2
        sample_size = min(width, height) // 4
        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)
        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size // 2)
        center_region = image[start_y:end_y, start_x:end_x]
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        return np.mean(hsv_region, axis=(0, 1))

    def process_frame(self, color_image):
        """프레임 처리 및 객체 탐지"""
        roi_x1, roi_y1, roi_x2, roi_y2 = Config.ROI_POSITION
        roi_image = color_image[roi_y1:roi_y2, roi_x1:roi_x2]
        results = self.yolo_model(roi_image)

        detection_results = []
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = map(int, result[:6])
            object_roi = roi_image[y1:y2, x1:x2]
            center_color = self.get_center_color(object_roi)
            color_name = self.get_color_name(center_color)
            label = self.yolo_model.names[class_id]
            detection_results.append((label, color_name, x1, y1, x2, y2))
        return detection_results

    def annotate_frame(self, color_image, detection_results):
        """탐지 결과를 프레임에 시각화"""
        roi_x1, roi_y1, _, _ = Config.ROI_POSITION
        for label, color_name, x1, y1, x2, y2 in detection_results:
            cv2.rectangle(color_image, (roi_x1 + x1, roi_y1 + y1), (roi_x1 + x2, roi_y1 + y2), (0, 255, 0), 2)
            cv2.putText(color_image, f'{label}-{color_name}', (roi_x1 + x1, roi_y1 + y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # ------------------------------
    # ROS 타이머 콜백
    # ------------------------------
    def timer_callback(self):
        """주기적으로 호출: 프레임 처리, 시각화 및 결과 전송"""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        detection_results = self.process_frame(color_image)
        self.annotate_frame(color_image, detection_results)
        self.send_results(detection_results)

        # 이미지 퍼블리시
        ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_publisher.publish(ros_image_message)

    # ------------------------------
    # 리소스 해제
    # ------------------------------
    def destroy_node(self):
        """리소스 해제"""
        self.pipeline.stop()
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectPanelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
