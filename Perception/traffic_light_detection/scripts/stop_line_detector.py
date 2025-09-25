#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import os
import rospkg
import math
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32

class StopLineDetector:
    def __init__(self):
        rospy.init_node('stop_line_detector', anonymous=True)
        
        # Parameters: topics
        self.odometry_topic = rospy.get_param('~odometry_topic', '/odometry')
        self.path_topic = rospy.get_param('~path_topic', '/planning/global/path')
        self.flag_topic = rospy.get_param('~flag_topic', '/perception/traffic_light/stop_line/flag')
        self.distance_topic = rospy.get_param('~distance_topic', '/perception/traffic_light/stop_line/distance')

        # Timer placeholder (avoid race when latched Path arrives before timer creation)
        self.timer = None

        # Subscribers
        self.sub_odom = rospy.Subscriber(self.odometry_topic, Odometry, self.cb_odom, queue_size=20)
        self.sub_path = rospy.Subscriber(self.path_topic, Path, self.cb_path, queue_size=1)

        # Publishers (standard only)
        self.stop_line_flag_pub = rospy.Publisher(self.flag_topic, Bool, queue_size=10)
        self.stop_line_distance_pub = rospy.Publisher(self.distance_topic, Float32, queue_size=10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.msg_received = False
        self.path = None
        self.path_received = False
        self.path_points = []   # [(x,y), ...]
        self.cumdist = []       # cumulative distance along path
        self.prev_vehicle_idx = None
        self.search_window = int(rospy.get_param('~search_window', 50))
        self.stop_idx_map = []  # [{'idx': int, 'file': str, 'x': float, 'y': float}]
        
        # 로컬 좌표계 기준점 (gps_to_local_cartesian.cpp와 동일)
        self.ref_lat = 37.338817432893464
        self.ref_lon = 127.89867211359652
        
        # 간단한 평면 근사를 위한 상수
        self.R_EARTH = 6378137.0
        
        # Legacy CSV-based stopline list (disabled by default)
        self.stop_lines = []
        self.csv_stoplines_enabled = bool(rospy.get_param('~csv_stoplines/enabled', False))
        if self.csv_stoplines_enabled:
            self.load_stop_lines()
        else:
            rospy.loginfo("CSV-based stopline loading disabled (~csv_stoplines/enabled:=false)")
        
        # 경로상 거리 기준 감지 거리
        self.detection_distance = 20.0  # 경로상 30m 이내
        
        rospy.loginfo("Stop Line Detector Initialized")
        rospy.loginfo(f"Reference point: Lat={self.ref_lat}, Lon={self.ref_lon}")
        rospy.loginfo("Waiting for /odometry and /planning/global/path topics...")
        
        # 타이머로 주기적으로 상태 체크
        self.timer = rospy.Timer(rospy.Duration(2.0), self.check_connection)
        
    def load_stop_lines(self):
        # Resolve CSV directory via parameter with sane default to this package's data directory
        default_data_dir = os.path.join(rospkg.RosPack().get_path('traffic_light_detection'), 'data')
        # Backward-compat param name kept (~csv_data_dir); namespaced variant also accepted
        gps_data_path = rospy.get_param('~csv_stoplines/csv_data_dir', rospy.get_param('~csv_data_dir', default_data_dir))

        csv_files = ['1.csv', '2.csv', '3.csv']  # 3번 정지선 추가
        
        for csv_file in csv_files:
            file_path = os.path.join(gps_data_path, csv_file)
            try:
                with open(file_path, 'r') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        lat = float(row['Lat'])
                        lon = float(row['Long'])
                        
                        # lat/lon을 로컬 좌표계로 변환 (gps_to_local_cartesian.cpp와 동일한 방식)
                        lat_r = math.radians(lat)
                        lon_r = math.radians(lon)
                        ref_lat_r = math.radians(self.ref_lat)
                        ref_lon_r = math.radians(self.ref_lon)
                        
                        local_x = (lon_r - ref_lon_r) * math.cos(ref_lat_r) * self.R_EARTH
                        local_y = (lat_r - ref_lat_r) * self.R_EARTH
                        
                        self.stop_lines.append({'x': local_x, 'y': local_y, 'file': csv_file})
                        rospy.loginfo(f"Loaded stop line from {csv_file}: Lat={lat:.8f}, Lon={lon:.8f} -> Local X={local_x:.2f}, Y={local_y:.2f}")
            except Exception as e:
                # Legacy assets may not exist in current setups; downgrade to warning
                rospy.logwarn(f"CSV stopline load skipped for {csv_file}: {e}")
                
    def check_connection(self, event):
        if not self.msg_received:
            rospy.logwarn(f"No {self.odometry_topic} messages received yet.")
        if not self.path_received:
            rospy.logwarn(f"No {self.path_topic} messages received yet.")
    
    def cb_odom(self, msg):
        if not self.msg_received:
            rospy.loginfo(f"✅ First {self.odometry_topic} message received!")
            self.msg_received = True

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.path_received:
            self.check_stop_lines()
    
    def cb_path(self, msg):
        if not self.path_received:
            rospy.loginfo(f"✅ First {self.path_topic} message received with {len(msg.poses)} points!")
            self.path_received = True
            if self.msg_received and getattr(self, 'timer', None) is not None:
                try:
                    self.timer.shutdown()  # 모든 연결 확인되면 타이머 종료
                except Exception:
                    pass
        
        self.path = msg
        self._rebuild_path_cache()
    
    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def _rebuild_path_cache(self):
        """Path 포인트 리스트 및 누적거리, 정지선 인덱스 매핑 캐시 생성"""
        if not self.path or len(self.path.poses) == 0:
            self.path_points = []
            self.cumdist = []
            self.stop_idx_map = []
            self.prev_vehicle_idx = None
            return

        pts = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
        self.path_points = pts
        self.cumdist = [0.0] * len(pts)
        for i in range(1, len(pts)):
            x1, y1 = pts[i-1]
            x2, y2 = pts[i]
            self.cumdist[i] = self.cumdist[i-1] + self.calculate_distance(x1, y1, x2, y2)

        # CSV 정지선이 있다면 경로 인덱스 미리 매핑
        self.stop_idx_map = []
        if self.stop_lines:
            for sl in self.stop_lines:
                idx, d = self.find_closest_point_on_path(sl['x'], sl['y'])
                if idx is not None and d <= 5.0:
                    self.stop_idx_map.append({'idx': idx, 'file': sl['file'], 'x': sl['x'], 'y': sl['y']})
        self.prev_vehicle_idx = None

    def find_closest_point_on_path(self, x, y):
        """경로상에서 주어진 좌표와 가장 가까운 점의 인덱스 찾기(캐시/윈도우 사용)"""
        if not self.path_points:
            return None, float('inf')
        if self.prev_vehicle_idx is None:
            start, end = 0, len(self.path_points)
        else:
            start = max(0, self.prev_vehicle_idx - self.search_window)
            end = min(len(self.path_points), self.prev_vehicle_idx + self.search_window + 1)

        min_dist = float('inf')
        closest_idx = start
        for i in range(start, end):
            px, py = self.path_points[i]
            d = self.calculate_distance(x, y, px, py)
            if d < min_dist:
                min_dist = d
                closest_idx = i
        return closest_idx, min_dist
    
    def calculate_path_distance(self, idx1, idx2):
        """두 인덱스 사이의 경로상 거리 계산(누적거리 이용)"""
        if not self.cumdist or idx1 is None or idx2 is None:
            return float('inf')
        if idx1 >= idx2:
            return -1.0
        idx2 = min(idx2, len(self.cumdist)-1)
        idx1 = max(0, idx1)
        return self.cumdist[idx2] - self.cumdist[idx1]
    
    def check_stop_lines(self):
        if not self.path_received or not self.path:
            return
        
        # 현재 차량 위치에서 경로상 가장 가까운 점 찾기 (캐시/윈도우)
        vehicle_idx, vehicle_dist = self.find_closest_point_on_path(self.current_x, self.current_y)
        if vehicle_idx is not None:
            self.prev_vehicle_idx = vehicle_idx
        
        if vehicle_idx is None or vehicle_dist > 5.0:  # 경로에서 5m 이상 떨어진 경우 무시
            rospy.logwarn_throttle(5.0, f"Vehicle is far from path: {vehicle_dist:.2f}m")
            stop_msg = Bool()
            stop_msg.data = False
            self.stop_line_flag_pub.publish(stop_msg)
            return
        
        flag = False
        min_path_distance = float('inf')
        closest_stop_line = None
        closest_stop_idx = None
        
        # 각 정지선에 대해 경로상 거리 계산 (사전 매핑 활용)
        iter_list = self.stop_idx_map if self.stop_idx_map else (
            [{'idx': self.find_closest_point_on_path(sl['x'], sl['y'])[0], 'file': sl['file'], 'x': sl['x'], 'y': sl['y']} for sl in self.stop_lines]
        )
        for item in iter_list:
            stop_idx = item.get('idx')
            if stop_idx is None:
                continue
            path_distance = self.calculate_path_distance(vehicle_idx, stop_idx)
            if path_distance > 0 and path_distance < min_path_distance:
                min_path_distance = path_distance
                closest_stop_line = {'file': item.get('file', 'csv'), 'x': item.get('x', 0.0), 'y': item.get('y', 0.0)}
                closest_stop_idx = stop_idx
        
        # 경로상 거리 기준으로 정지선 감지
        if closest_stop_line and min_path_distance <= self.detection_distance:
            flag = True
            rospy.loginfo_throttle(1.0, f"⚠️ Stop line ahead from {closest_stop_line['file']}: Path distance={min_path_distance:.2f}m")
            
            # 정지선까지의 거리 발행
            dist_msg = Float32()
            dist_msg.data = max(0.5, min_path_distance)  # 최소 0.5m 보장
            self.stop_line_distance_pub.publish(dist_msg)
        elif closest_stop_line:
            rospy.loginfo_throttle(2.0, f"Nearest stop line: {closest_stop_line['file']} - Path distance: {min_path_distance:.2f}m")
        
        stop_msg = Bool()
        stop_msg.data = flag
        self.stop_line_flag_pub.publish(stop_msg)
        
def main():
    try:
        detector = StopLineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
