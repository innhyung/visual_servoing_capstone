#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math

class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        
        # 1. 리얼센스 전용 토픽 구독
        # color는 compressed(JPEG)로 받아 대역폭/버퍼링 절감, depth는 raw 유지
        self.color_sub = self.create_subscription(
            CompressedImage, '/camera/camera/color/image_raw/compressed',
            self.color_callback, 10)

        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        # 2. 로봇팔 제어 토픽 퍼블리셔
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') 
        
        # 제어 게인값 (선속도 전용)
        self.kp_linear = 0.0015
        self.target_object = 'mouse'

        # 안전 한계 (m/s) — 이 값 넘으면 servo가 singularity로 들어가 NaN 발생
        self.max_linear = 0.08
        self.search_radius_max = 0.04
        
        self.cv_depth_image = None 
        
        # memory 변수 추가
        self.last_bx = 0
        self.last_by = 0
        self.last_x1 = 0
        self.last_y1 = 0
        self.last_x2 = 0
        self.last_y2 = 0
        
        # 상태 머신 변수
        self.state = 'SEARCHING' # 초기 상태: 탐색
        self.miss_count = 0
        self.search_angle = 0.0
        self.search_radius = 0.01

        # servo가 첫 callback 직후엔 IK 준비 안 돼 NaN을 뱉을 수 있어
        # 처음 N프레임은 publish를 건너뛴다 (warmup)
        self.warmup_frames = 10
        self.frame_count = 0
        
        self.get_logger().info("RealSense D455 비주얼 서보잉...")

    def depth_callback(self, msg):
        self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        
    def color_callback(self, msg):
        # CompressedImage(JPEG) → OpenCV BGR
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            return
        height, width = cv_image.shape[:2]
        cx, cy = width // 2, height // 2

        results = self.model(cv_image, verbose=False)
        target_found = False
        cmd_msg = TwistStamped()
        
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link' # 로봇 몸통 기준 제어
        
        # 기본적으로 손목 각도(Angular)는 꺾이지 않도록 0으로
        cmd_msg.twist.angular.x = 0.0
        cmd_msg.twist.angular.y = 0.0
        cmd_msg.twist.angular.z = 0.0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_name = self.model.names[int(box.cls[0])]
                if cls_name == self.target_object:
                    target_found = True
                    self.miss_count = 0 
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    self.last_x1, self.last_y1 = x1, y1
                    self.last_x2, self.last_y2 = x2, y2
                    self.last_bx, self.last_by = (x1 + x2) // 2, (y1 + y2) // 2
                    break 
            if target_found: break

        # 타겟 상실 카운트
        if not target_found:
            self.miss_count += 1
            if self.miss_count > 30 and self.state == 'SERVOING':
                self.state = 'SEARCHING'
                self.search_angle = 0.0
                self.search_radius = 0.01
                self.get_logger().warn("타겟 상실... 나선형 탐색 시작")

        
        # State별 로봇팔 행동 제어
                
        # [상태 1] 탐색 모드 (Spiral Search)
        if self.state == 'SEARCHING':
            if target_found:
                self.state = 'SERVOING'
                self.get_logger().info("타겟 포착... 추적 시작")
            else:
                # 나선형 궤적을 그리려면 위치(R·cosθ)가 아닌 속도(미분값)를 보내야 함
                # 원 위치: x=R·cos(ωt), y=R·sin(ωt)
                # 필요한 속도: vx=-R·ω·sin(ωt), vy=R·ω·cos(ωt)
                omega = 1.5  # rad/s  (R_max·ω = 0.04·1.5 = 0.06 m/s < max_linear=0.08)
                self.search_angle += omega / 30.0   # 약 30fps 기준
                self.search_radius = min(self.search_radius + 0.0003, self.search_radius_max)
                cmd_msg.twist.linear.x = -self.search_radius * omega * math.sin(self.search_angle)
                cmd_msg.twist.linear.y =  self.search_radius * omega * math.cos(self.search_angle)
                cv2.putText(cv_image, "Mode: SEARCHING", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

        # [상태 2] 추적 모드 (Visual Servoing)
        elif self.state == 'SERVOING':
            error_x = self.last_bx - cx
            error_y = self.last_by - cy
            
            # 오차를 각속도가 아닌 선속도(Linear)에 곱해서 평행 이동
            cmd_msg.twist.linear.x = float(error_x) * self.kp_linear
            cmd_msg.twist.linear.y = -float(error_y) * self.kp_linear
            
            cv2.rectangle(cv_image, (self.last_x1, self.last_y1), (self.last_x2, self.last_y2), (0, 255, 0), 2)
            cv2.line(cv_image, (cx, cy), (self.last_bx, self.last_by), (0, 255, 255), 2)
            cv2.putText(cv_image, "Mode: SERVOING", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            distance_mm = 0
            if self.cv_depth_image is not None and 0 <= self.last_by < height and 0 <= self.last_bx < width:
                distance_mm = self.cv_depth_image[self.last_by, self.last_bx]
                
                if distance_mm > 0:
                    cv2.putText(cv_image, f"Z: {distance_mm}mm", (self.last_bx+10, self.last_by-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    
                    # 호버링 조건: 오차가 15픽셀 이내이고, D455 데드존(350mm)에 근접한 400mm 도달 시 정지
                    if abs(error_x) < 15 and abs(error_y) < 15 and distance_mm <= 400:
                        self.state = 'HOVERING'
                        self.get_logger().info("정렬 성공... Z축 블라인드 드롭(파지) 준비")

        # [상태 3] 호버링 및 파지 (Blind Drop)
        elif self.state == 'HOVERING':
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.linear.y = 0.0
            cmd_msg.twist.linear.z = -0.05 # 평행 이동 멈추고 수직으로 
            
            cv2.putText(cv_image, "Mode: BLIND DROP...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        # warmup: 처음 몇 프레임은 publish 건너뛰기 (servo 초기화 시간 확보)
        self.frame_count += 1
        if self.frame_count <= self.warmup_frames:
            cv2.putText(cv_image, f"Warmup {self.frame_count}/{self.warmup_frames}",
                        (50, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        else:
            # 안전 가드: NaN/Inf 검사 + max_linear로 clip
            lin = cmd_msg.twist.linear
            ang = cmd_msg.twist.angular
            vals = [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
            if not all(np.isfinite(v) for v in vals):
                self.get_logger().warn(f"Non-finite cmd 차단: {vals}")
            else:
                lin.x = float(np.clip(lin.x, -self.max_linear, self.max_linear))
                lin.y = float(np.clip(lin.y, -self.max_linear, self.max_linear))
                lin.z = float(np.clip(lin.z, -self.max_linear, self.max_linear))
                self.publisher_.publish(cmd_msg)

        cv2.imshow("RealSense VLA", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
