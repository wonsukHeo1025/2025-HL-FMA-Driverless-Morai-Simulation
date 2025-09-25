# **Phase 4: 고급 기능 구현 및 시스템 최적화**

## **1. 개요**

본 문서는 Lateral MPC 시스템의 고급 기능 구현, 성능 최적화, 그리고 실차 적용을 위한 강건성 향상 계획을 정의합니다. Phase 1-3에서 구축된 기본 시스템을 바탕으로, Pure Pursuit 대비 우수한 성능을 달성하고 실제 환경에서의 안정적인 동작을 보장하는 것이 목표입니다.

**핵심 목표**:
- **적응형 제어**: 속도와 도로 조건에 따른 동적 파라미터 조정
- **예측 성능 향상**: 도로 곡률 예측 및 차량 동역학 보상
- **실시간 최적화**: C++ 구현 및 병렬 처리로 연산 시간 단축
- **강건성 확보**: 센서 노이즈, 모델 불확실성 대응

## **2. 3단계 검증 워크플로우**

### **2.1. 검증 단계 구성**

```text
[Stage 1: 이상적 환경]
├─ 상태: Perfect state from TF
├─ 경로: Ideal path without noise
├─ 목표: MPC 로직 및 모델 검증
└─ KPI: Lateral RMSE < 0.1m

[Stage 2: 현실적 센서]
├─ 상태: GPS/IMU fusion
├─ 경로: Filtered path
├─ 목표: 상태 추정 성능 검증
└─ KPI: Lateral RMSE < 0.2m

[Stage 3: 완전 통합]
├─ 상태: Noisy sensors
├─ 경로: Real-time planning
├─ 목표: 시스템 강건성 검증
└─ KPI: Lateral RMSE < 0.3m
```

### **2.2. Stage 1: 제어기 코어 검증**

```python
class Stage1Validator:
    """이상적 환경에서 MPC 코어 로직 검증"""
    
    def __init__(self):
        self.test_scenarios = {
            'straight': self.validate_straight_tracking,
            'constant_radius': self.validate_circular_tracking,
            'sine_wave': self.validate_sine_tracking,
            'step_change': self.validate_step_response
        }
        
    def validate_straight_tracking(self):
        """직선 경로 추종 (기준선 설정)"""
        # 초기 오프셋 테스트
        initial_offsets = [0.5, 1.0, 1.5]  # meters
        
        for offset in initial_offsets:
            # 경로에서 offset만큼 떨어진 위치에서 시작
            metrics = self.run_test(
                path_type='straight',
                initial_lateral_offset=offset,
                duration=10.0
            )
            
            # 수렴 시간 확인
            assert metrics['settling_time'] < 3.0, \
                f"Settling time {metrics['settling_time']}s exceeds 3s"
            
            # 정상상태 오차 확인
            assert metrics['steady_state_error'] < 0.05, \
                f"Steady state error {metrics['steady_state_error']}m exceeds 0.05m"
                
    def validate_circular_tracking(self):
        """정상상태 선회 검증"""
        test_cases = [
            {'radius': 30, 'speed': 30},  # 느린 속도, 작은 반경
            {'radius': 30, 'speed': 50},  # 빠른 속도, 작은 반경
            {'radius': 50, 'speed': 50},  # 빠른 속도, 큰 반경
        ]
        
        for case in test_cases:
            metrics = self.run_test(
                path_type='circle',
                radius=case['radius'],
                speed=case['speed'],
                duration=20.0
            )
            
            # 언더스티어 보상 확인
            theoretical_steering = self.calculate_ackermann_steering(
                case['radius'], self.wheelbase
            )
            understeer_compensation = metrics['avg_steering'] - theoretical_steering
            
            print(f"R={case['radius']}m, V={case['speed']}km/h: "
                  f"Understeer compensation = {np.degrees(understeer_compensation):.1f}°")
```

### **2.3. Stage 2: 센서 융합 검증**

```python
class Stage2Validator:
    """현실적 센서 데이터로 상태 추정 검증"""
    
    def __init__(self):
        self.sensor_configs = {
            'gps_only': {'gps_noise': 0.5, 'imu_noise': 0},
            'imu_only': {'gps_noise': float('inf'), 'imu_noise': 0.01},
            'fusion': {'gps_noise': 0.5, 'imu_noise': 0.01}
        }
        
    def validate_state_estimation(self):
        """상태 추정 정확도 검증"""
        
        for config_name, noise_params in self.sensor_configs.items():
            # 센서 노이즈 적용
            self.apply_sensor_noise(noise_params)
            
            # 테스트 실행
            metrics = self.run_test_with_config(config_name)
            
            # Ground truth (TF) 대비 오차
            position_rmse = metrics['position_rmse']
            velocity_rmse = metrics['velocity_rmse']
            
            print(f"{config_name}: Pos RMSE={position_rmse:.3f}m, "
                  f"Vel RMSE={velocity_rmse:.3f}m/s")
            
            # 허용 오차 확인
            if config_name == 'fusion':
                assert position_rmse < 0.2, "Position estimation error too large"
                assert velocity_rmse < 0.5, "Velocity estimation error too large"
```

### **2.4. Stage 3: 통합 시스템 강건성**

```python
class Stage3Validator:
    """완전 통합 시스템의 강건성 검증"""
    
    def __init__(self):
        self.disturbances = {
            'sensor_dropout': self.test_sensor_dropout,
            'model_mismatch': self.test_model_mismatch,
            'actuator_delay': self.test_actuator_delay,
            'external_disturbance': self.test_wind_disturbance
        }
        
    def test_sensor_dropout(self):
        """센서 신호 손실 대응"""
        dropout_scenarios = [
            {'sensor': 'gps', 'duration': 2.0},
            {'sensor': 'imu', 'duration': 1.0},
            {'sensor': 'both', 'duration': 0.5}
        ]
        
        for scenario in dropout_scenarios:
            # 센서 드롭아웃 시뮬레이션
            metrics = self.simulate_dropout(scenario)
            
            # 시스템이 안정적으로 유지되는지 확인
            assert not metrics['control_saturation'], \
                f"Control saturated during {scenario['sensor']} dropout"
            assert metrics['max_lateral_error'] < 1.0, \
                f"Excessive error during {scenario['sensor']} dropout"
                
    def test_model_mismatch(self):
        """모델 파라미터 불확실성 테스트"""
        parameter_variations = [
            {'param': 'mass', 'variation': 0.2},      # ±20% 질량 변화
            {'param': 'cornering_stiffness', 'variation': 0.3},  # ±30% 타이어 특성
            {'param': 'wheelbase', 'variation': 0.05}  # ±5% 휠베이스
        ]
        
        for variation in parameter_variations:
            # 파라미터 변화 적용
            metrics = self.test_with_parameter_variation(variation)
            
            # 성능 저하 허용 범위 확인
            performance_degradation = metrics['rmse_increase']
            assert performance_degradation < 0.5, \
                f"Excessive degradation with {variation['param']} mismatch"
```

## **3. 적응형 MPC 구현**

### **3.1. 속도 적응형 파라미터**

```python
class AdaptiveMPC(LateralMpcCore):
    """속도에 따라 동적으로 조정되는 MPC"""
    
    def __init__(self, base_params):
        super().__init__(base_params)
        self.speed_ranges = {
            'low': (0, 30),     # 0-30 km/h
            'medium': (30, 50),  # 30-50 km/h
            'high': (50, 80)     # 50-80 km/h
        }
        
    def adapt_parameters(self, current_speed):
        """속도에 따른 파라미터 조정"""
        
        # 속도 구간 판별
        speed_kmh = current_speed * 3.6
        
        if speed_kmh < 30:
            # 저속: 정밀한 추종
            self.control_params['Q'][0, 0] = 150  # 높은 횡방향 가중치
            self.control_params['preview_distance'] = 3.0
            self.control_params['Np'] = 15
            
        elif speed_kmh < 50:
            # 중속: 균형잡힌 제어
            self.control_params['Q'][0, 0] = 100
            self.control_params['preview_distance'] = 5.0
            self.control_params['Np'] = 20
            
        else:
            # 고속: 안정성 우선
            self.control_params['Q'][0, 0] = 80
            self.control_params['Q'][2, 2] = 100  # 헤딩 중시
            self.control_params['preview_distance'] = 8.0
            self.control_params['Np'] = 25
            self.control_params['R'] = 2.0  # 부드러운 조향
```

### **3.2. 도로 조건 적응**

```python
class RoadAdaptiveMPC(AdaptiveMPC):
    """도로 곡률과 마찰 계수를 고려한 적응형 MPC"""
    
    def adapt_to_road_conditions(self, road_info):
        """도로 조건에 따른 적응"""
        
        # 곡률 기반 조정
        avg_curvature = np.mean(np.abs(road_info['curvatures']))
        
        if avg_curvature > 0.05:  # 급커브 구간
            self.control_params['R_delta'] *= 1.5  # 급조향 억제
            self.control_params['delta_rate_max'] *= 0.8
            
        # 노면 마찰 계수 반영
        if road_info.get('friction_coefficient', 1.0) < 0.7:
            # 미끄러운 노면
            self.model_params['Caf'] *= road_info['friction_coefficient']
            self.model_params['Car'] *= road_info['friction_coefficient']
            self.control_params['delta_limits'] = [-0.5, 0.5]  # 조향각 제한
            
    def predict_road_curvature(self, path, horizon):
        """예측 구간의 도로 곡률 예측"""
        
        curvature_profile = []
        for k in range(horizon):
            lookahead = self.control_params['preview_distance'] + \
                       k * self.control_params['Ts'] * self.current_speed
            
            # 곡률 계산 및 필터링
            raw_curvature = self.calculate_curvature_at(path, lookahead)
            filtered_curvature = self.kalman_filter.update(raw_curvature)
            curvature_profile.append(filtered_curvature)
            
        return np.array(curvature_profile)
```

## **4. 성능 최적화**

### **4.1. C++ 구현**

```cpp
// lateral_mpc_core.cpp

#include <Eigen/Dense>
#include <osqp/osqp.h>

class LateralMpcCore {
public:
    LateralMpcCore(const ModelParams& model_params, 
                   const ControlParams& control_params)
        : model_params_(model_params), 
          control_params_(control_params) {
        setupOptimizationProblem();
    }
    
    double solve(const Eigen::VectorXd& current_state,
                 const std::vector<double>& reference_curvatures,
                 double current_speed) {
        
        // Update model matrices for current speed
        updateModelMatrices(current_speed);
        
        // Update QP matrices
        updateQPMatrices(current_state, reference_curvatures);
        
        // Solve QP problem
        osqp_solve(workspace_);
        
        // Extract optimal control
        return workspace_->solution->x[0];
    }
    
private:
    void setupOptimizationProblem() {
        // Pre-allocate matrices
        int n_states = 4;
        int n_controls = control_params_.Nc;
        int n_constraints = calculateNumConstraints();
        
        // Initialize OSQP workspace
        P_data_.reserve(n_controls * n_controls);
        A_data_.reserve(n_constraints * n_controls);
        
        // Setup sparse matrices
        // ... (implementation)
    }
    
    void updateModelMatrices(double vx) {
        // Dynamic model linearization
        double a21 = -(model_params_.Caf + model_params_.Car) / 
                     (model_params_.m * vx);
        double a24 = (-model_params_.lf * model_params_.Caf + 
                      model_params_.lr * model_params_.Car) / 
                     (model_params_.m * vx);
        // ... update A and B matrices
    }
};
```

### **4.2. 병렬 처리 최적화**

```python
import multiprocessing as mp
from concurrent.futures import ThreadPoolExecutor

class ParallelMPC:
    """병렬 처리를 활용한 MPC 최적화"""
    
    def __init__(self, num_threads=4):
        self.executor = ThreadPoolExecutor(max_workers=num_threads)
        
    def parallel_prediction(self, initial_states, control_sequences):
        """여러 제어 시퀀스를 병렬로 시뮬레이션"""
        
        futures = []
        for control_seq in control_sequences:
            future = self.executor.submit(
                self.simulate_trajectory,
                initial_states,
                control_seq
            )
            futures.append(future)
            
        # 결과 수집
        trajectories = [f.result() for f in futures]
        
        # 최적 시퀀스 선택
        costs = [self.calculate_cost(traj) for traj in trajectories]
        best_idx = np.argmin(costs)
        
        return control_sequences[best_idx]
```

### **4.3. GPU 가속 (CUDA)**

```python
import cupy as cp

class GPUAcceleratedMPC:
    """GPU를 활용한 대규모 행렬 연산 가속"""
    
    def __init__(self):
        self.use_gpu = cp.cuda.is_available()
        
    def batch_matrix_multiply(self, A_batch, B_batch):
        """배치 행렬 곱셈 GPU 가속"""
        
        if self.use_gpu:
            A_gpu = cp.asarray(A_batch)
            B_gpu = cp.asarray(B_batch)
            C_gpu = cp.matmul(A_gpu, B_gpu)
            return cp.asnumpy(C_gpu)
        else:
            return np.matmul(A_batch, B_batch)
```

## **5. 장애물 회피 통합**

### **5.1. 장애물 정보 처리**

```python
class ObstacleAwareMPC(LateralMpcCore):
    """장애물 회피를 고려한 MPC"""
    
    def __init__(self, base_params):
        super().__init__(base_params)
        self.obstacle_buffer = []
        
    def add_obstacle_constraints(self, obstacles, predicted_path):
        """장애물 회피 제약 조건 추가"""
        
        constraints = []
        for obstacle in obstacles:
            for k in range(self.control_params['Np']):
                # 예측 시점 k에서의 차량 위치
                vehicle_pos = predicted_path[k]
                
                # 장애물과의 거리 제약
                distance = np.linalg.norm(
                    vehicle_pos - obstacle['position']
                )
                min_distance = obstacle['radius'] + self.safety_margin
                
                # Soft constraint로 추가
                constraints.append(distance >= min_distance)
                
        return constraints
    
    def compute_collision_risk(self, trajectory, obstacles):
        """충돌 위험도 계산"""
        
        risk = 0.0
        for k, pos in enumerate(trajectory):
            for obs in obstacles:
                distance = np.linalg.norm(pos - obs['position'])
                
                # 거리 기반 위험도 (지수 감소)
                risk += np.exp(-distance / obs['influence_radius'])
                
        return risk
```

## **6. 안전 기능 구현**

### **6.1. Fail-Safe 메커니즘**

```python
class SafeMPC:
    """안전 기능이 강화된 MPC"""
    
    def __init__(self):
        self.fallback_controller = PurePursuitController()
        self.health_monitor = SystemHealthMonitor()
        
    def safe_control(self, state, path):
        """안전한 제어 명령 생성"""
        
        try:
            # 시스템 상태 확인
            if not self.health_monitor.is_healthy():
                return self.fallback_control(state, path)
                
            # MPC 최적화
            steering = self.mpc_core.solve(state, path)
            
            # 제어 입력 검증
            if not self.validate_control(steering):
                return self.fallback_control(state, path)
                
            return steering
            
        except Exception as e:
            rospy.logerr(f"MPC failed: {e}")
            return self.fallback_control(state, path)
            
    def validate_control(self, steering):
        """제어 입력 유효성 검증"""
        
        # 물리적 한계 확인
        if abs(steering) > self.max_steering:
            return False
            
        # 급격한 변화 확인
        if hasattr(self, 'last_steering'):
            delta = abs(steering - self.last_steering)
            if delta > self.max_steering_rate * self.dt:
                return False
                
        return True
```

### **6.2. 성능 모니터링 및 로깅**

```python
class PerformanceLogger:
    """상세 성능 로깅 및 분석"""
    
    def __init__(self, log_dir='/tmp/lateral_mpc_logs'):
        self.log_dir = log_dir
        self.metrics_buffer = []
        self.start_time = rospy.Time.now()
        
    def log_iteration(self, state, control, metrics):
        """매 제어 주기 로깅"""
        
        timestamp = (rospy.Time.now() - self.start_time).to_sec()
        
        log_entry = {
            'timestamp': timestamp,
            'state': state.tolist(),
            'control': control,
            'lateral_error': metrics['lateral_error'],
            'heading_error': metrics['heading_error'],
            'solve_time': metrics['solve_time'],
            'cost': metrics['cost']
        }
        
        self.metrics_buffer.append(log_entry)
        
        # 주기적으로 파일로 저장
        if len(self.metrics_buffer) >= 1000:
            self.flush_to_file()
            
    def generate_report(self):
        """성능 분석 리포트 생성"""
        
        df = pd.DataFrame(self.metrics_buffer)
        
        report = {
            'total_duration': df['timestamp'].max(),
            'avg_lateral_error': df['lateral_error'].mean(),
            'max_lateral_error': df['lateral_error'].max(),
            'rmse_lateral': np.sqrt((df['lateral_error']**2).mean()),
            'avg_solve_time': df['solve_time'].mean(),
            'max_solve_time': df['solve_time'].max(),
            'control_smoothness': df['control'].diff().std()
        }
        
        return report
```

## **7. 비교 평가 프레임워크**

### **7.1. Pure Pursuit vs MPC 벤치마크**

```python
class BenchmarkSuite:
    """체계적인 성능 비교 테스트"""
    
    def __init__(self):
        self.test_tracks = {
            'oval': self.generate_oval_track,
            'figure8': self.generate_figure8_track,
            'monaco': self.load_monaco_track,
            'nurburgring': self.load_nurburgring_track
        }
        
    def run_benchmark(self, controller_type, track_name):
        """단일 벤치마크 실행"""
        
        track = self.test_tracks[track_name]()
        
        # 초기 조건 설정
        initial_state = self.get_initial_state(track)
        
        # 시뮬레이션 실행
        start_time = time.time()
        trajectory, metrics = self.simulate(
            controller_type,
            track,
            initial_state,
            duration=60.0
        )
        end_time = time.time()
        
        # 결과 분석
        results = {
            'controller': controller_type,
            'track': track_name,
            'lap_time': self.calculate_lap_time(trajectory),
            'avg_error': metrics['lateral_error'].mean(),
            'max_error': metrics['lateral_error'].max(),
            'computation_time': end_time - start_time,
            'energy_usage': self.calculate_energy(metrics['control'])
        }
        
        return results
    
    def compare_controllers(self):
        """전체 비교 분석"""
        
        results = []
        
        for track_name in self.test_tracks:
            # Pure Pursuit
            pp_result = self.run_benchmark('pure_pursuit', track_name)
            results.append(pp_result)
            
            # Kinematic MPC
            kmpc_result = self.run_benchmark('kinematic_mpc', track_name)
            results.append(kmpc_result)
            
            # Dynamic MPC
            dmpc_result = self.run_benchmark('dynamic_mpc', track_name)
            results.append(dmpc_result)
            
        # 결과 테이블 생성
        df = pd.DataFrame(results)
        
        # 상대 성능 계산
        for metric in ['lap_time', 'avg_error', 'max_error']:
            pp_baseline = df[df['controller'] == 'pure_pursuit'][metric]
            df[f'{metric}_improvement'] = (pp_baseline - df[metric]) / pp_baseline * 100
            
        return df
```

## **8. 실차 적용 준비**

### **8.1. Hardware-in-the-Loop (HIL) 테스트**

```yaml
# config/hil_test_config.yaml
hil_configuration:
  # 실제 ECU 연결
  can_interface:
    device: can0
    baudrate: 500000
    
  # 센서 시뮬레이션
  sensor_simulation:
    gps:
      update_rate: 10  # Hz
      noise_std: 0.5   # meters
    imu:
      update_rate: 100 # Hz
      noise_std: 0.01  # rad/s
      
  # 액추에이터 모델
  steering_actuator:
    max_rate: 30      # deg/s
    delay: 0.05       # seconds
    deadband: 0.5     # degrees
```

### **8.2. 실차 캘리브레이션**

```python
class VehicleCalibration:
    """실차 파라미터 캘리브레이션"""
    
    def calibrate_steering_map(self):
        """조향 명령과 실제 바퀴 각도 매핑"""
        
        calibration_points = []
        
        for cmd in np.linspace(-1.0, 1.0, 21):
            # 조향 명령 전송
            self.send_steering_command(cmd)
            rospy.sleep(1.0)
            
            # 실제 조향각 측정
            actual_angle = self.measure_wheel_angle()
            
            calibration_points.append({
                'command': cmd,
                'actual_angle': actual_angle
            })
            
        # 매핑 함수 생성
        self.steering_map = self.fit_polynomial(calibration_points)
        
    def validate_model_parameters(self):
        """모델 파라미터 실차 검증"""
        
        validation_tests = [
            self.validate_wheelbase,
            self.validate_cornering_stiffness,
            self.validate_yaw_inertia
        ]
        
        results = {}
        for test in validation_tests:
            param_name, measured_value, model_value = test()
            error_percent = abs(measured_value - model_value) / model_value * 100
            
            results[param_name] = {
                'measured': measured_value,
                'model': model_value,
                'error_percent': error_percent
            }
            
        return results
```

## **9. 최종 성능 목표 및 검증**

### **9.1. 성능 목표**

| 메트릭 | Pure Pursuit | Lateral MPC (달성) | 개선율 |
|--------|--------------|-------------------|--------|
| 횡방향 RMSE (@40km/h) | 0.45m | 0.15m | 67% |
| 최대 오차 (@40km/h) | 1.1m | 0.4m | 64% |
| 언더스티어 보상 | 없음 | 자동 | - |
| 계산 시간 | <1ms | <15ms | - |
| 곡선 구간 오차 | 0.8m | 0.25m | 69% |

### **9.2. 최종 검증 체크리스트**

- [ ] **기능 검증**
  - [ ] Kinematic/Dynamic 모델 전환
  - [ ] 속도 적응형 파라미터
  - [ ] 장애물 회피 통합
  - [ ] Fail-safe 동작

- [ ] **성능 검증**
  - [ ] 실시간 제약 만족 (50Hz)
  - [ ] 모든 테스트 트랙에서 목표 달성
  - [ ] 센서 노이즈 강건성
  - [ ] 모델 불확실성 대응

- [ ] **통합 검증**
  - [ ] 전체 자율주행 스택 통합
  - [ ] 장시간 주행 안정성
  - [ ] 비상 상황 대응
  - [ ] 실차 테스트 준비 완료

## **10. 프로젝트 완료 및 인수인계**

### **10.1. 문서화**

```markdown
## Lateral MPC Controller Documentation

### Quick Start
1. Launch: `roslaunch lateral_mpc lateral_mpc_controller.launch`
2. Monitor: `rosrun plotjuggler plotjuggler`
3. Tune: Edit `config/mpc_params.yaml`

### Architecture
- Core: `lateral_mpc_core.py` - MPC optimization
- Node: `lateral_mpc_controller_node.py` - ROS interface
- Utils: Path processing, state estimation

### Key Parameters
- Q: State error weights [e_y, ė_y, e_ψ, ė_ψ]
- R: Control effort weight
- Np/Nc: Prediction/Control horizons
```

### **10.2. 유지보수 가이드**

| 이슈 | 진단 | 해결 |
|------|------|------|
| 추종 성능 저하 | 로그 분석 | 파라미터 재튜닝 |
| 계산 시간 증가 | 프로파일링 | 호라이즌 축소 |
| 모델 부정확 | 검증 테스트 | 재식별 필요 |
| 센서 드리프트 | 칼만 필터 확인 | 공분산 조정 |

## **11. 결론**

Phase 4 완료 시 Lateral MPC 시스템은:
- Pure Pursuit 대비 60% 이상의 성능 개선 달성
- 다양한 속도와 도로 조건에서 안정적 동작
- 실시간 제약 조건 만족 (50Hz @ <15ms)
- 실차 적용 가능한 수준의 강건성 확보

이로써 차세대 자율주행 차량을 위한 고성능 횡방향 제어 시스템 개발이 완료됩니다.