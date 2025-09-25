#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO

try:
    import torch
    _TORCH_AVAILABLE = True
except Exception:
    _TORCH_AVAILABLE = False

class TrafficLightDetector:
    def __init__(self):
        rospy.init_node('traffic_light_detector', anonymous=True)

        # 클래스 이름과 색상 정의
        self.class_names = ['green', 'red', 'yellow']
        self.class_colors = {
            'green': (0, 255, 0),
            'red': (0, 0, 255),
            'yellow': (0, 255, 255)
        }

        # 파라미터: 토픽/추론/로깅 (ROI 제거)
        self.image_topic = rospy.get_param('~image_topic', '/image_jpeg/compressed')
        self.state_topic = rospy.get_param('~state_topic', '/perception/traffic_light/state')

        device_param = str(rospy.get_param('~device', 'cpu')).strip().lower()
        self.half = bool(rospy.get_param('~half', False))
        self.imgsz = int(rospy.get_param('~imgsz', 640))
        self.conf_thres = float(rospy.get_param('~conf', 0.5))
        self.iou_thres = float(rospy.get_param('~iou', 0.5))

        self.enable_viz = bool(rospy.get_param('~enable_viz', False))
        self.log_throttle = float(rospy.get_param('~log_throttle', 1.0))

        # Debug image publishing (annotated with YOLO boxes)
        self.publish_debug_image = bool(rospy.get_param('~publish_debug_image', True))
        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/perception/traffic_light/annotated/compressed')

        # YOLOv8 모델 로드
        model_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data', 'best.pt')
        if not os.path.exists(model_path):
            rospy.logerr(f"Model file not found at: {model_path}")
            rospy.signal_shutdown("Model file not found")
            return

        self.model = YOLO(model_path)

        # 디바이스/FP16 설정
        self.device = 'cpu'
        if _TORCH_AVAILABLE:
            if device_param.startswith('cuda') or device_param in ('gpu', '0'):
                if torch.cuda.is_available():
                    self.device = 'cuda:0' if device_param in ('gpu', '0', 'cuda') else device_param
                else:
                    rospy.logwarn("CUDA requested but not available; falling back to CPU")
            else:
                self.device = 'cpu'

            try:
                self.model.to(self.device)
            except Exception as e:
                rospy.logwarn(f"Model.to({self.device}) failed: {e}")

            # CPU에서는 half 비활성화
            if self.device == 'cpu':
                self.half = False

            # conv+bn fuse (가능한 경우)
            try:
                _ = self.model.fuse()
            except Exception:
                pass

            if self.device.startswith('cuda'):
                try:
                    torch.backends.cudnn.benchmark = True
                except Exception:
                    pass

        rospy.loginfo(f"Loaded YOLOv8 model from: {model_path} | device={self.device}, half={self.half}, imgsz={self.imgsz}, conf={self.conf_thres}, iou={self.iou_thres}")

        # 이미지 subscriber (기본 버퍼 설정 사용)
        self.image_sub = rospy.Subscriber(self.image_topic,
                                          CompressedImage,
                                          self.image_callback,
                                          queue_size=1)

        # 신호등 상태 publisher (표준만)
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=10)

        # 디버그 이미지 publisher (압축 이미지)
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = rospy.Publisher(self.debug_image_topic, CompressedImage, queue_size=1)

        rospy.loginfo("Traffic Light Detector initialized")

    def image_callback(self, msg):
        try:
            # Compressed 이미지를 OpenCV 이미지로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logwarn_throttle(2.0, "Failed to decode image")
                return
            
            # ROI 크롭 제거: 전체 이미지로 추론
            infer_img = cv_image

            # YOLOv8로 추론 (벡터화 후처리)
            if _TORCH_AVAILABLE:
                ctx = torch.inference_mode()
            else:
                # 더미 컨텍스트
                class _Dummy:
                    def __enter__(self, *a):
                        return self
                    def __exit__(self, *a):
                        return False
                ctx = _Dummy()

            with ctx:
                results = self.model(
                    infer_img,
                    conf=self.conf_thres,
                    iou=self.iou_thres,
                    imgsz=self.imgsz,
                    device=self.device if _TORCH_AVAILABLE else None,
                    half=self.half if _TORCH_AVAILABLE else False,
                    verbose=False,
                )

            best_detection = None
            max_confidence = 0.0

            # 주석 렌더링 여부 결정 (RViz/화면 표시(enable_viz) 또는 디버그 토픽 발행)
            want_annotate = self.enable_viz or (self.publish_debug_image and self.debug_image_pub is not None)
            annotated_image = cv_image.copy() if want_annotate else None

            result = results[0] if isinstance(results, (list, tuple)) and len(results) > 0 else results
            boxes = getattr(result, 'boxes', None)

            if boxes is not None and hasattr(boxes, 'xyxy') and boxes.xyxy is not None:
                # 한 번만 CPU로 내리고 넘파이로 변환
                xyxy = boxes.xyxy
                conf = boxes.conf
                cls = boxes.cls
                try:
                    xyxy = xyxy.cpu().numpy().astype(int)
                    conf = conf.view(-1).cpu().numpy()
                    cls = cls.view(-1).cpu().numpy().astype(int)
                except Exception:
                    # CPU 환경 등에서 .cpu() 불필요한 구현 대비
                    xyxy = np.asarray(xyxy).astype(int)
                    conf = np.asarray(conf).reshape(-1)
                    cls = np.asarray(cls).astype(int).reshape(-1)

                # 최상 신뢰도 선택
                if conf.size > 0:
                    best_idx = int(np.argmax(conf))
                    cid = int(cls[best_idx])
                    if 0 <= cid < len(self.class_names):
                        best_detection = self.class_names[cid]
                        max_confidence = float(conf[best_idx])

                # 시각화/디버깅 이미지가 켜진 경우 바운딩 박스/레이블 그리기
                if want_annotate and xyxy.size > 0 and annotated_image is not None:
                    for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
                        if 0 <= k < len(self.class_names):
                            name = self.class_names[k]
                            color = self.class_colors.get(name, (255, 255, 255))
                            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
                            label = f"{name}: {float(c):.2f}"
                            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                            cv2.rectangle(annotated_image, (x1, y1 - th - 5), (x1 + tw, y1), color, -1)
                            cv2.putText(annotated_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    # 필요시 이미지 디스플레이/저장 로직 추가 가능

            # 가장 신뢰도가 높은 신호등 클래스를 토픽으로 발행
            if best_detection:
                out = String(data=best_detection)
                self.state_pub.publish(out)
                rospy.loginfo_throttle(self.log_throttle, f"Published traffic light state: {out.data} (conf: {max_confidence:.2f})")

            # 디버깅용 주석 이미지를 토픽으로 발행 (감지 없어도 원본 그대로 발행)
            if self.publish_debug_image and self.debug_image_pub is not None and annotated_image is not None:
                try:
                    ok, enc = cv2.imencode('.jpg', annotated_image)
                    if ok:
                        dbg = CompressedImage()
                        dbg.header = msg.header
                        dbg.format = 'jpeg'
                        dbg.data = enc.tobytes()
                        self.debug_image_pub.publish(dbg)
                except Exception as e:
                    rospy.logwarn_throttle(2.0, f"Failed to publish annotated image: {e}")
            
        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")
    
    def run(self):
        rospy.spin()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == '__main__':
    try:
        detector = TrafficLightDetector()
        detector.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
