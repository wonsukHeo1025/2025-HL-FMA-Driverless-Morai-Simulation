# **Phase 3: ROS 노드 구현 및 시스템 통합**

## **1. 개요**

본 문서는 `Phase 2`에서 설계된 `LateralMpcCore` 제어기를 실제 MORAI 시뮬레이터 환경과 통합하기 위한 ROS 노드 구현 및 시스템 통합 계획을 정의합니다.

핵심 목표는 경로 정보와 차량 상태를 실시간으로 처리하여 최적 조향 명령을 생성하는 완전한 제어 시스템을 구축하는 것입니다.

**통합 목표**:
- Pure Pursuit와의 성능 비교를 위한 동일한 인터페이스 제공
- 다양한 상태 추정 소스 지원 (TF, GPS/IMU, Odometry)
- 실시간 모니터링 및 디버깅 인프라 구축
- PlotJuggler를 통한 성능 분석 지원

## **2. 시스템 아키텍처**

### **2.1. 노드 구성도**

```text
                    +----------------------------------+
                    |    lateral_mpc_controller_node   |
                    +----------------------------------+
                              |            |
                    +---------v--+    +----v---------+
                    | State       |    | Path         |
                    | Estimator   |    | Processor    |
                    +-------------+    +--------------+
                           |                  |
                    +------v------------------v-------+
                    |      LateralMpcCore             |
                    |   (from Phase 2)                |
                    +----------------------------------+
                              |
                    +---------v---------+
                    | Control Command   |
                    | Publisher         |
                    +-------------------+
                              |
                              v
                        [/ctrl_cmd]
```

### **2.2. 데이터 플로우**

```text
Inputs:
├── /global_path (nav_msgs/Path) - 전역 경로
├── /local_path (nav_msgs/Path) - 지역 경로 (선택)
├── /Competition_topic (morai_msgs/EgoVehicleStatus) - 차량 상태
├── /tf (tf2_msgs/TFMessage) - 좌표 변환
└── /gps, /imu (센서 데이터)

Processing:
├── Path → Curvature, Reference Points
├── Vehicle State → Current Errors (e_y, e_psi)
└── MPC Optimization → Optimal Steering

Outputs:
├── /ctrl_cmd (morai_msgs/CtrlCmd) - 제어 명령
├── /lateral_mpc/debug (custom msg) - 디버그 정보
├── /lateral_mpc/predicted_path (nav_msgs/Path) - 예측 경로
└── /lateral_mpc/metrics (std_msgs/Float32MultiArray) - 성능 지표
```

## **3. ROS 노드 상세 설계**

### **3.1. 노드 구조**

```python
#!/usr/bin/env python3
# lateral_mpc_controller_node.py

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf2_geometry_msgs

from lateral_mpc.lateral_mpc_core import LateralMpcCore
from lateral_mpc.path_processor import PathProcessor
from lateral_mpc.state_estimator import StateEstimator

class LateralMpcControllerNode:
    def __init__(self):
        rospy.init_node('lateral_mpc_controller')
        
        # 파라미터 로드
        self._load_parameters()
        
        # MPC 코어 초기화
        self.mpc_core = LateralMpcCore(
            self.model_params,
            self.control_params
        )
        
        # 헬퍼 클래스 초기화
        self.path_processor = PathProcessor()
        self.state_estimator = StateEstimator(self.state_source)
        
        # Publishers
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.debug_pub = rospy.Publisher('/lateral_mpc/debug', 
                                        LateralMpcDebug, queue_size=1)
        self.pred_path_pub = rospy.Publisher('/lateral_mpc/predicted_path', 
                                            Path, queue_size=1)
        self.metrics_pub = rospy.Publisher('/lateral_mpc/metrics', 
                                          Float32MultiArray, queue_size=1)
        
        # Subscribers
        self.path_sub = rospy.Subscriber('/global_path', Path, 
                                        self.path_callback)
        self.ego_sub = rospy.Subscriber('/Competition_topic', 
                                       EgoVehicleStatus, 
                                       self.ego_callback)
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 제어 타이머 (50Hz)
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0/self.control_rate), 
            self.control_loop
        )
        
        # 상태 변수
        self.current_path = None
        self.current_speed = 0.0
        self.last_steering = 0.0
        
        rospy.loginfo("Lateral MPC Controller initialized")
```

### **3.2. 파라미터 관리**

```python
def _load_parameters(self):
    """ROS 파라미터 서버에서 설정 로드"""
    
    # 제어 파라미터
    self.control_rate = rospy.get_param('~control_rate', 50.0)
    self.state_source = rospy.get_param('~state_source', 'tf')
    
    # MPC 파라미터
    self.control_params = {
        'Np': rospy.get_param('~mpc/prediction_horizon', 20),
        'Nc': rospy.get_param('~mpc/control_horizon', 5),
        'Ts': 1.0 / self.control_rate,
        'Q': self._parse_matrix(rospy.get_param('~mpc/Q', 
                                               [100, 10, 50, 5])),
        'R': rospy.get_param('~mpc/R', 1.0),
        'R_delta': rospy.get_param('~mpc/R_delta', 10.0),
        'delta_limits': rospy.get_param('~mpc/delta_limits', 
                                       [-0.7, 0.7]),
        'delta_rate_max': rospy.get_param('~mpc/delta_rate_max', 0.01),
        'preview_distance': rospy.get_param('~mpc/preview_distance', 5.0)
    }
    
    # 모델 파라미터
    model_mode = rospy.get_param('~model/mode', 'kinematic')
    
    if model_mode == 'kinematic':
        self.model_params = {
            'mode': 'kinematic',
            'L': rospy.get_param('~model/wheelbase', 2.875)
        }
    else:
        self.model_params = {
            'mode': 'dynamic',
            'm': rospy.get_param('~model/mass', 1800),
            'Iz': rospy.get_param('~model/yaw_inertia', 2500),
            'lf': rospy.get_param('~model/lf', 1.3),
            'lr': rospy.get_param('~model/lr', 1.575),
            'Caf': rospy.get_param('~model/Caf', 100000),
            'Car': rospy.get_param('~model/Car', 120000),
            'Kv': rospy.get_param('~model/understeer_gradient', 0.002)
        }
```

### **3.3. 경로 처리**

```python
class PathProcessor:
    """경로 정보 처리 및 곡률 계산"""
    
    def __init__(self):
        self.path_points = []
        self.curvatures = []
        
    def update_path(self, path_msg):
        """ROS Path 메시지를 처리"""
        self.path_points = []
        for pose_stamped in path_msg.poses:
            point = {
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'heading': self._quaternion_to_yaw(
                    pose_stamped.pose.orientation
                )
            }
            self.path_points.append(point)
            
        self._calculate_curvatures()
        
    def _calculate_curvatures(self):
        """각 경로 포인트에서의 곡률 계산"""
        n = len(self.path_points)
        self.curvatures = np.zeros(n)
        
        for i in range(1, n-1):
            # 3점을 이용한 곡률 계산
            p1 = self.path_points[i-1]
            p2 = self.path_points[i]
            p3 = self.path_points[i+1]
            
            self.curvatures[i] = self._three_point_curvature(p1, p2, p3)
            
    def get_reference_at_distance(self, current_pose, distances):
        """
        현재 위치에서 지정된 거리들에서의 참조점 반환
        """
        references = []
        
        # 가장 가까운 경로 포인트 찾기
        closest_idx = self._find_closest_point(current_pose)
        
        for dist in distances:
            ref_idx = self._get_index_at_distance(closest_idx, dist)
            if ref_idx < len(self.path_points):
                references.append({
                    'point': self.path_points[ref_idx],
                    'curvature': self.curvatures[ref_idx]
                })
                
        return references
    
    def calculate_errors(self, current_pose):
        """현재 위치에서의 경로 추종 오차 계산"""
        closest_idx = self._find_closest_point(current_pose)
        
        if closest_idx >= len(self.path_points) - 1:
            return 0, 0  # 경로 끝
            
        # 횡방향 오차 (Cross-track error)
        path_point = self.path_points[closest_idx]
        next_point = self.path_points[closest_idx + 1]
        
        # 경로 접선 벡터
        path_vec = np.array([next_point['x'] - path_point['x'],
                           next_point['y'] - path_point['y']])
        path_heading = np.arctan2(path_vec[1], path_vec[0])
        
        # 차량 위치에서 경로까지의 벡터
        to_vehicle = np.array([current_pose['x'] - path_point['x'],
                              current_pose['y'] - path_point['y']])
        
        # 횡방향 오차 (부호 포함)
        e_y = np.cross(path_vec / np.linalg.norm(path_vec), 
                      to_vehicle[:2])
        
        # 헤딩 오차
        e_psi = self._normalize_angle(
            current_pose['heading'] - path_heading
        )
        
        return e_y, e_psi
```

### **3.4. 상태 추정**

```python
class StateEstimator:
    """차량 상태 추정 (다양한 소스 지원)"""
    
    def __init__(self, source='tf'):
        self.source = source
        self.current_state = {
            'x': 0, 'y': 0, 'heading': 0,
            'vx': 0, 'vy': 0, 'yaw_rate': 0
        }
        
        if source == 'gps_imu':
            self._init_gps_imu_fusion()
        elif source == 'odometry':
            self._init_odometry_subscriber()
            
    def update_from_tf(self, tf_buffer):
        """TF에서 상태 추정"""
        try:
            # map -> base_link 변환
            transform = tf_buffer.lookup_transform(
                'map', 'base_link', 
                rospy.Time(0), 
                rospy.Duration(0.1)
            )
            
            # 위치
            self.current_state['x'] = transform.transform.translation.x
            self.current_state['y'] = transform.transform.translation.y
            
            # 헤딩
            q = transform.transform.rotation
            self.current_state['heading'] = self._quaternion_to_yaw(q)
            
            # 속도 (유한차분)
            if hasattr(self, 'last_state'):
                dt = (rospy.Time.now() - self.last_time).to_sec()
                if dt > 0:
                    self.current_state['vx'] = (
                        self.current_state['x'] - self.last_state['x']
                    ) / dt
                    self.current_state['vy'] = (
                        self.current_state['y'] - self.last_state['y']
                    ) / dt
                    self.current_state['yaw_rate'] = (
                        self.current_state['heading'] - 
                        self.last_state['heading']
                    ) / dt
                    
            self.last_state = self.current_state.copy()
            self.last_time = rospy.Time.now()
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            
    def update_from_ego_status(self, ego_msg):
        """MORAI EgoVehicleStatus에서 속도 정보 업데이트"""
        self.current_state['vx'] = ego_msg.velocity.x
        self.current_state['vy'] = ego_msg.velocity.y
        # Note: MORAI heading은 비어있으므로 global_yaw_estimator 필요
        
    def get_error_state(self, path_processor):
        """경로 추종 오차 상태 벡터 반환"""
        e_y, e_psi = path_processor.calculate_errors(self.current_state)
        
        if self.model_mode == 'kinematic':
            return np.array([e_y, e_psi])
        else:
            # Dynamic model: [e_y, ė_y, e_psi, ė_psi]
            return np.array([
                e_y,
                self.current_state['vy'] + 
                self.current_state['vx'] * e_psi,  # ė_y ≈ vy + vx*e_psi
                e_psi,
                self.current_state['yaw_rate']  # ė_psi ≈ yaw_rate
            ])
```

### **3.5. 제어 루프**

```python
def control_loop(self, event):
    """메인 제어 루프 (50Hz)"""
    
    # 경로가 없으면 대기
    if self.current_path is None:
        return
        
    # 1. 상태 업데이트
    if self.state_source == 'tf':
        self.state_estimator.update_from_tf(self.tf_buffer)
    
    # 2. 오차 상태 계산
    error_state = self.state_estimator.get_error_state(
        self.path_processor
    )
    
    # 3. 예견 거리에서의 참조 경로 정보
    Np = self.control_params['Np']
    Ts = self.control_params['Ts']
    v_x = self.current_speed
    
    preview_distances = [
        self.control_params['preview_distance'] + i * Ts * v_x 
        for i in range(Np)
    ]
    references = self.path_processor.get_reference_at_distance(
        self.state_estimator.current_state,
        preview_distances
    )
    
    # 4. MPC 최적화
    optimal_steering, predicted_traj = self.mpc_core.solve(
        error_state,
        references,
        v_x
    )
    
    # 5. 제어 명령 발행
    self.publish_control_command(optimal_steering)
    
    # 6. 디버그 정보 발행
    self.publish_debug_info(
        error_state, 
        optimal_steering, 
        predicted_traj
    )
    
    # 7. 성능 메트릭 업데이트
    self.update_metrics(error_state, optimal_steering)
    
def publish_control_command(self, steering_angle):
    """제어 명령 발행"""
    cmd = CtrlCmd()
    cmd.longlCmdType = 2  # Velocity control
    cmd.velocity = self.current_speed  # 속도는 유지
    cmd.steering = np.degrees(steering_angle)  # rad -> deg
    cmd.accel = 0
    cmd.brake = 0
    
    self.ctrl_pub.publish(cmd)
    self.last_steering = steering_angle
```

### **3.6. 디버깅 및 모니터링**

```python
def publish_debug_info(self, error_state, steering, predicted_traj):
    """디버그 정보 발행 (PlotJuggler용)"""
    
    # Custom debug message
    debug_msg = LateralMpcDebug()
    debug_msg.header.stamp = rospy.Time.now()
    
    # 현재 오차
    debug_msg.lateral_error = error_state[0]
    if len(error_state) >= 3:
        debug_msg.heading_error = error_state[2]
    
    # 제어 입력
    debug_msg.steering_command = steering
    debug_msg.steering_rate = (steering - self.last_steering) * \
                              self.control_rate
    
    # MPC 내부 상태
    debug_msg.cost_value = self.mpc_core.last_cost
    debug_msg.solver_time = self.mpc_core.last_solve_time
    debug_msg.solver_status = self.mpc_core.last_status
    
    self.debug_pub.publish(debug_msg)
    
    # 예측 경로 발행
    if predicted_traj is not None:
        pred_path = Path()
        pred_path.header.frame_id = "map"
        pred_path.header.stamp = rospy.Time.now()
        
        for i in range(predicted_traj.shape[1]):
            pose = PoseStamped()
            # 예측된 오차를 실제 위치로 변환
            # ... (구현)
            pred_path.poses.append(pose)
            
        self.pred_path_pub.publish(pred_path)
        
def update_metrics(self, error_state, steering):
    """성능 메트릭 계산 및 발행"""
    
    # 메트릭 누적
    if not hasattr(self, 'metrics_buffer'):
        self.metrics_buffer = {
            'lateral_errors': [],
            'heading_errors': [],
            'steering_commands': []
        }
    
    self.metrics_buffer['lateral_errors'].append(error_state[0])
    if len(error_state) >= 3:
        self.metrics_buffer['heading_errors'].append(error_state[2])
    self.metrics_buffer['steering_commands'].append(steering)
    
    # 주기적으로 통계 계산 (1Hz)
    if len(self.metrics_buffer['lateral_errors']) >= self.control_rate:
        metrics = Float32MultiArray()
        metrics.data = [
            np.sqrt(np.mean(np.square(
                self.metrics_buffer['lateral_errors']
            ))),  # Lateral RMSE
            np.max(np.abs(
                self.metrics_buffer['lateral_errors']
            )),  # Max lateral error
            np.sqrt(np.mean(np.square(
                self.metrics_buffer['heading_errors']
            ))),  # Heading RMSE
            np.std(np.diff(
                self.metrics_buffer['steering_commands']
            ))  # Steering smoothness
        ]
        
        self.metrics_pub.publish(metrics)
        
        # 버퍼 초기화
        for key in self.metrics_buffer:
            self.metrics_buffer[key] = []
```

## **4. Launch 파일 구성**

### **4.1. 기본 실행**

```xml
<!-- lateral_mpc_controller.launch -->
<launch>
    <!-- MPC Controller Node -->
    <node name="lateral_mpc_controller" 
          pkg="lateral_mpc" 
          type="lateral_mpc_controller_node.py" 
          output="screen">
          
        <!-- Control Parameters -->
        <param name="control_rate" value="50.0"/>
        <param name="state_source" value="tf"/>
        
        <!-- Model Selection -->
        <param name="model/mode" value="kinematic"/>
        <param name="model/wheelbase" value="2.875"/>
        
        <!-- MPC Parameters -->
        <param name="mpc/prediction_horizon" value="20"/>
        <param name="mpc/control_horizon" value="5"/>
        <rosparam param="mpc/Q">[100, 10, 50, 5]</rosparam>
        <param name="mpc/R" value="1.0"/>
        <param name="mpc/R_delta" value="10.0"/>
        <rosparam param="mpc/delta_limits">[-0.7, 0.7]</rosparam>
        <param name="mpc/preview_distance" value="5.0"/>
    </node>
    
    <!-- Optional: Path Visualizer -->
    <node name="path_visualizer" 
          pkg="lateral_mpc" 
          type="path_visualizer.py">
        <remap from="path" to="/global_path"/>
    </node>
    
    <!-- Optional: Performance Monitor -->
    <node name="performance_monitor" 
          pkg="lateral_mpc" 
          type="performance_monitor.py"/>
</launch>
```

### **4.2. Dynamic Model 실행**

```xml
<!-- lateral_mpc_dynamic.launch -->
<launch>
    <include file="$(find lateral_mpc)/launch/lateral_mpc_controller.launch">
        <!-- Override to Dynamic Model -->
        <arg name="model_mode" value="dynamic"/>
        
        <!-- Dynamic Model Parameters (from Phase 1) -->
        <arg name="model_mass" value="1800"/>
        <arg name="model_yaw_inertia" value="2500"/>
        <arg name="model_lf" value="1.3"/>
        <arg name="model_lr" value="1.575"/>
        <arg name="model_Caf" value="100000"/>
        <arg name="model_Car" value="120000"/>
    </include>
</launch>
```

## **5. 테스트 및 검증**

### **5.1. 단위 테스트**

```python
# test/test_lateral_mpc_node.py

import unittest
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TestLateralMpcNode(unittest.TestCase):
    
    def setUp(self):
        rospy.init_node('test_lateral_mpc', anonymous=True)
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        
    def test_straight_path_tracking(self):
        """직선 경로 추종 테스트"""
        # 직선 경로 생성
        path = self.create_straight_path(100, 1.0)
        self.path_pub.publish(path)
        
        # 제어 명령 확인
        # ... (구현)
        
    def test_circular_path_tracking(self):
        """원형 경로 추종 테스트"""
        # R=30m 원형 경로 생성
        path = self.create_circular_path(30, 100)
        self.path_pub.publish(path)
        
        # 정상상태 조향각 확인
        # ... (구현)
        
    def test_state_estimation_switching(self):
        """상태 추정 소스 전환 테스트"""
        # TF → GPS/IMU 전환
        # ... (구현)
```

### **5.2. 통합 테스트 시나리오**

```python
# scripts/integration_test.py

class IntegrationTest:
    """Pure Pursuit vs MPC 비교 테스트"""
    
    def __init__(self):
        self.test_scenarios = [
            'straight_line',
            'constant_radius_30m',
            'double_lane_change',
            's_curve',
            'sharp_corner'
        ]
        
    def run_comparison(self):
        """각 시나리오에서 두 제어기 비교"""
        results = {}
        
        for scenario in self.test_scenarios:
            # Pure Pursuit 실행
            pp_metrics = self.run_pure_pursuit(scenario)
            
            # Lateral MPC 실행
            mpc_metrics = self.run_lateral_mpc(scenario)
            
            # 결과 비교
            results[scenario] = {
                'pure_pursuit': pp_metrics,
                'lateral_mpc': mpc_metrics,
                'improvement': self.calculate_improvement(
                    pp_metrics, mpc_metrics
                )
            }
            
        return results
```

## **6. 성능 모니터링 도구**

### **6.1. PlotJuggler 설정**

```yaml
# config/plotjuggler_lateral_mpc.xml
<root>
  <tabbed_widget name="Main" parent="main_window">
    <Tab name="Tracking Errors" tab_index="0">
      <DockArea>
        <plot name="Lateral Error" row="0" col="0">
          <curve name="/lateral_mpc/debug/lateral_error"/>
          <curve name="/pure_pursuit/lateral_error"/>
        </plot>
        <plot name="Heading Error" row="0" col="1">
          <curve name="/lateral_mpc/debug/heading_error"/>
        </plot>
      </DockArea>
    </Tab>
    <Tab name="Control Inputs" tab_index="1">
      <DockArea>
        <plot name="Steering Command" row="0" col="0">
          <curve name="/lateral_mpc/debug/steering_command"/>
        </plot>
        <plot name="Steering Rate" row="1" col="0">
          <curve name="/lateral_mpc/debug/steering_rate"/>
        </plot>
      </DockArea>
    </Tab>
    <Tab name="MPC Performance" tab_index="2">
      <DockArea>
        <plot name="Solver Time" row="0" col="0">
          <curve name="/lateral_mpc/debug/solver_time"/>
        </plot>
        <plot name="Cost Value" row="0" col="1">
          <curve name="/lateral_mpc/debug/cost_value"/>
        </plot>
      </DockArea>
    </Tab>
  </tabbed_widget>
</root>
```

### **6.2. 실시간 성능 대시보드**

```python
# scripts/performance_dashboard.py

class PerformanceDashboard:
    """웹 기반 실시간 모니터링"""
    
    def __init__(self):
        self.app = Flask(__name__)
        self.metrics = {}
        
        # ROS Subscribers
        rospy.Subscriber('/lateral_mpc/metrics', 
                        Float32MultiArray, 
                        self.metrics_callback)
        
    def metrics_callback(self, msg):
        """메트릭 업데이트"""
        self.metrics = {
            'lateral_rmse': msg.data[0],
            'max_lateral_error': msg.data[1],
            'heading_rmse': msg.data[2],
            'steering_smoothness': msg.data[3],
            'timestamp': rospy.Time.now().to_sec()
        }
        
    @app.route('/metrics')
    def get_metrics(self):
        """REST API for metrics"""
        return jsonify(self.metrics)
```

## **7. 문제 해결 가이드**

| 문제 | 가능한 원인 | 해결 방법 |
|------|------------|----------|
| 제어 명령 미발행 | 경로 수신 안됨 | `/global_path` 토픽 확인 |
| 큰 추종 오차 | 잘못된 상태 추정 | state_source 변경 |
| 조향 떨림 | MPC 가중치 부적절 | R, R_delta 증가 |
| 느린 연산 | 큰 예측 호라이즌 | Np, Nc 감소 |
| TF 오류 | TF tree 미완성 | tf tree 확인 |

## **8. 다음 단계 (Phase 4 준비)**

Phase 3에서 구현된 시스템은 Phase 4에서:
- 고급 기능 추가 (적응형 MPC, 장애물 회피)
- 성능 최적화 (C++ 포팅, GPU 가속)
- 실차 적용을 위한 안전 기능
- 강건성 향상 (센서 노이즈 대응)