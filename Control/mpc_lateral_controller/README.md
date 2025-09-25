# MPC Lateral Controller

## 패키지 개요

Dynamic Bicycle Model 기반 횡방향 MPC 컨트롤러입니다. Error Dynamics 정식화를 통해 경로 추종 성능을 최적화합니다.

### 🚀 OSQP 백엔드 지원
고성능 실시간 제어를 위한 직접 OSQP 구현을 지원합니다:
- **120-200배 속도 향상**: CVXPY 대비 획기적인 성능 개선
- **실시간 제어 가능**: 모든 구성에서 50Hz 이상 달성 가능
- **수치 정확도 유지**: CVXPY와 1e-5 이내 일치

자세한 최적화 계획은 [OSQP_Optimization_Plan.md](OSQP_Optimization_Plan.md) 참조

### 식별된 파라미터 (하드코딩)
```python
m = 1901.0        # 차량 질량 [kg]
Iz = 2456.54      # yaw 관성모멘트 [kg·m²] 
lf = 1.7          # 전륜 거리 [m]
lr = 1.3          # 후륜 거리 [m]
Caf = 50040.6     # 전륜 코너링 강성 [N/rad]
Car = 198123.4    # 후륜 코너링 강성 [N/rad]
Kv = 0.00684      # understeer gradient [rad/(m/s²)]
```

## 패키지 구조

```
mpc_lateral_controller/
├── scripts/
│   ├── lateral_mpc_controller_node.py   # 메인 ROS 노드
│   ├── path_visualizer.py               # RViz 시각화
│   └── performance_monitor.py           # 성능 모니터링
├── src/mpc_lateral_controller/
│   ├── lateral_mpc_core.py              # MPC 최적화 코어
│   ├── path_processor.py                # 경로 처리
│   └── state_estimator.py              # 상태 추정
├── launch/
│   ├── lateral_mpc_controller.launch    # 기본 실행
│   └── lateral_mpc_with_viz.launch     # 시각화 포함
└── config/
    └── mpc_params.yaml                  # MPC 파라미터
```

## 실행 방법

### 기본 실행
```bash
roslaunch mpc_lateral_controller lateral_mpc_controller.launch
```

### 시각화 포함 실행  
```bash
roslaunch mpc_lateral_controller lateral_mpc_with_viz.launch
```

### OSQP 백엔드로 실행 (고성능)
```bash
# Kinematic 모드 + OSQP
roslaunch mpc_lateral_controller lateral_mpc_controller.launch _mpc/solver_backend:=osqp _model_mode:=kinematic

# Dynamic 모드 + OSQP
roslaunch mpc_lateral_controller lateral_mpc_controller.launch _mpc/solver_backend:=osqp _model_mode:=dynamic

# 또는 launch 파일에서 직접 설정
<param name="mpc/solver_backend" value="osqp"/>
```

### Launch 파일 설명

1. **lateral_mpc_controller.launch**: 기본 MPC 컨트롤러만 실행
2. **lateral_mpc_with_viz.launch**: MPC + path_visualizer + performance_monitor 실행

## 작동 원리

### 1. Error Dynamics 모델

**상태 변수** (경로 기준 오차):
- $e_y$: 횡방향 오차 [m]
- $\dot{e}_y$: 횡방향 오차 변화율 [m/s]  
- $e_\psi$: 헤딩 오차 [rad]
- $\dot{e}_\psi$: 헤딩 오차 변화율 [rad/s]

**제어 입력**: $\delta$ (조향각 [rad])

**연속시간 시스템 모델**:
$$\dot{x} = A_c \cdot x + B_c \cdot u + E_c \cdot \kappa$$

여기서:
- $\kappa$: 경로 곡률 (외란으로 처리)
- $E_c = [0, -v_x^2, 0, 0]^T$: 곡률 외란 gain

**시스템 행렬**:
$$A_c = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{C_{af}+C_{ar}}{m \cdot v_x} & \frac{C_{af}+C_{ar}}{m} & \frac{-l_f \cdot C_{af}+l_r \cdot C_{ar}}{m \cdot v_x} \\
0 & 0 & 0 & 1 \\
0 & \frac{-l_f \cdot C_{af}+l_r \cdot C_{ar}}{I_z \cdot v_x} & \frac{l_f \cdot C_{af}-l_r \cdot C_{ar}}{I_z} & -\frac{l_f^2 \cdot C_{af}+l_r^2 \cdot C_{ar}}{I_z \cdot v_x}
\end{bmatrix}$$

$$B_c = \begin{bmatrix} 0 \\ \frac{C_{af}}{m} \\ 0 \\ \frac{l_f \cdot C_{af}}{I_z} \end{bmatrix}$$

### 2. MPC 정식화

**비용 함수**:
$$J = \sum_{k=0}^{N_p-1} (x_k^T Q x_k + u_k^T R u_k + \Delta u_k^T R_\delta \Delta u_k) + x_{N_p}^T P x_{N_p}$$

**제약 조건**:
- 조향각: $|\delta| \leq 0.7$ rad (±40°)
- 조향 변화율: $|\Delta\delta| \leq 0.01$ rad/step

**예측 모델**:
- 예측 구간(Np): 20 steps
- 제어 구간(Nc): 5 steps
- 샘플링 시간(Ts): 0.02s (50Hz)

### 3. Feedforward 보상

Dynamic 모델의 understeer 보상:
$$\delta_{ff} = L \cdot \kappa + K_v \cdot v_x^2 \cdot \kappa$$

- 첫 번째 항: 기하학적 조향
- 두 번째 항: understeer 보상

## 튜닝 가이드

### Q 행렬 조정 $[q_{e_y}, q_{\dot{e}_y}, q_{e_\psi}, q_{\dot{e}_\psi}]$

| 파라미터 | 기본값 | 증가시 효과 | 감소시 효과 |
|---------|--------|------------|------------|
| $q_{e_y}$ | 100 | 경로 추종 정밀도 ↑ | 부드러운 주행 |
| $q_{\dot{e}_y}$ | 10 | 횡방향 진동 감소 | 빠른 오차 수정 |
| $q_{e_\psi}$ | 50 | 헤딩 정렬 개선 | 경로 우선 추종 |
| $q_{\dot{e}_\psi}$ | 5 | 헤딩 변화 안정화 | 빠른 헤딩 수정 |

### 제어 가중치

- **$R$** (제어 노력): 
  - 증가 → 부드러운 조향
  - 감소 → 적극적인 제어
  - 기본값: 1.0
  
- **$R_\delta$** (제어 변화율):
  - 증가 → 조향 변화 완화
  - 감소 → 빠른 조향 응답
  - 기본값: 10.0

### 속도별 튜닝 전략

**저속 (< 30 km/h)**:
- $q_{e_y}$ ↑ (150~200): 정밀한 경로 추종
- $R$ ↓ (0.5~1.0): 적극적 조향

**고속 (> 60 km/h)**:
- $q_{e_\psi}$ ↑ (70~100): 안정적 헤딩 유지
- $R_\delta$ ↑ (15~20): 급조향 방지

## 주요 파라미터

### MPC 설정
```yaml
prediction_horizon: 20    # 예측 구간
control_horizon: 5        # 제어 구간  
control_rate: 50 Hz       # 제어 주기
preview_distance: 5.0 m   # 전방 주시 거리
```

### 토픽 인터페이스

**입력**:
- `/global_path` (nav_msgs/Path): 목표 경로
- `/Competition_topic` (morai_msgs/EgoVehicleStatus): 차량 상태
- `/imu` (sensor_msgs/Imu): IMU 데이터

**출력**:
- `/ctrl_cmd` (morai_msgs/CtrlCmd): 조향 명령
- `/lateral_mpc/debug` (LateralMpcDebug): 디버그 정보
- `/lateral_mpc/status` (LateralMpcStatus): 상태 정보
- `/lateral_mpc/predicted_path` (nav_msgs/Path): 예측 경로
- `/lateral_mpc/metrics` (Float32MultiArray): 성능 지표

## 디버깅

### 성능 모니터링
```bash
# 실시간 메트릭 확인
rostopic echo /lateral_mpc/metrics

# 디버그 정보 확인
rostopic echo /lateral_mpc/debug
```

### 시각화 (RViz)
- 예측 경로: `/lateral_mpc/predicted_path`
- 경로 마커: `/lateral_mpc/path_markers`
- 오차 마커: `/lateral_mpc/error_markers`

### 로그 확인
```bash
# 성능 로그 확인
tail -f /tmp/lateral_mpc_logs/performance.csv
```

## 알고리즘 특징

### 이산화 방법
- Zero-Order Hold (ZOH) 근사 사용
- 2차 Taylor 전개로 정확도 향상:
  $$A_d = I + A_c \cdot T_s + 0.5 \cdot A_c^2 \cdot T_s^2$$
  $$B_d = (I \cdot T_s + 0.5 \cdot A_c \cdot T_s^2) \cdot B_c$$

### 곡률 추정
- PathProcessor에서 3점 원 피팅 방법 사용
- 전방 주시 거리 기반 lookahead point 선택
- Smoothing filter 적용으로 노이즈 감소

### Solver 설정
- OSQP solver 사용 (실시간 성능 최적화)
- Warm start 활성화
- 평균 solving time: < 5ms

## 주의사항

1. **저속 주행시**: $v_x < 0.1$ m/s에서는 특이점 회피를 위해 최소 속도 0.1 m/s로 클리핑
2. **곡률 계산**: path_processor에서 3점 원 피팅으로 계산, 급격한 곡률 변화시 smoothing 필요
3. **상태 추정**: TF, ego_status, gps_imu 중 선택 가능 (기본: TF)
4. **초기화**: 첫 5초간은 시스템 안정화를 위해 제어 출력 제한

## 성능 최적화 팁

1. **실시간성 확보**:
   - control_rate를 시스템 성능에 맞게 조정 (30~50Hz 권장)
   - prediction_horizon 감소시 계산 부하 감소

2. **경로 추종 개선**:
   - 경로 샘플링 밀도 증가 (0.5~1.0m 간격)
   - preview_distance를 속도에 비례하게 조정

3. **안정성 향상**:
   - 고속에서 $R_\delta$ 증가
   - Terminal cost $(P) = Q \times 10$ 설정으로 안정성 확보

## 문제 해결: `/lateral_mpc/control_info` 출력 속도가 매우 느린 경우

증상: `/ctrl_cmd`는 60–80 Hz로 빠르게 갱신되는데 `/lateral_mpc/control_info`는 약 4 Hz로 드물게 나오고, 그때마다 큰 극단값이 튀며 제어 실패가 발생.

원인 분석:
- 제어 루프 내부에서 TF 조회가 블로킹되어 콜백이 정지됨. 기존 구현은 `lookup_transform(..., Duration(0.1))`로 최대 100 ms 대기 → 80 Hz 루프를 쉽게 막음.
- ROS TCP Nagle/버퍼링으로 메시지들이 묶여 전달됨. 기본 `tcp_nodelay=False`, 큐가 커서 지연 증가.
- 타이머 생성 시 period 정밀도와 콜백 내부 작업량이 불필요하게 커져 주기 지터 확대.

적용된 해결책:
- 비블로킹 TF 조회로 전환: 먼저 `can_transform(..., Duration(0.0))`로 가능 여부를 확인하고, 가능 시 `lookup_transform(..., Duration(0.0))` 사용. 실패 시 해당 틱을 건너뛰어 루프 주기 유지. 구현: `src/mpc_lateral_controller/state_estimator.py`의 `update_from_tf`.
- 전송 지연 축소: 핵심 퍼블리셔/서브스크라이버에 `tcp_nodelay=True`, `queue_size=1` 적용. 적용 대상: `/lateral_mpc/control_info`, `/lateral_mpc/debug`, `/lateral_mpc/status`, `/global_path`, `/Competition_topic`, `/imu`.
- 타이머 정밀도 개선: `rospy.Duration.from_sec(1.0/control_rate)`와 `oneshot=False`로 제어 타이머 생성. 실패 시 로깅 대신 틱 스킵.

검증 방법:
- `rostopic hz /lateral_mpc/control_info`가 60–80 Hz에 근접하는지 확인.
- `/lateral_mpc/debug`에서 `solver_time`과 `steering_rate`의 연속성 확인(시간 갭이 없어야 함).
- 여전히 느리다면 `/reference -> /base` TF의 발행 주기와 지연을 점검하고, 전역 yaw 추정 주기도 확인.

참고: 시뮬레이터 튜닝(공격적 매개변수)은 런치 파일의 `mpc/*` 오버라이드에서 조정할 수 있습니다.

## 참고: 글로벌 속도 계획 `max_speed`가 적용되지 않을 때

`path_roi_planner`의 `global_speed_planning`에서 `max_speed`를 2.0 m/s로 바꿔도 반영되지 않는다면, 상위 런치가 덮어쓰는 경우가 대부분입니다. `gps_global_planner/motion_planning_pipeline.launch`는 다음처럼 노드 파라미터를 설정합니다:

```xml
<arg name="max_speed" default="20.0"/>
...
<node name="global_speed_planning" pkg="path_roi_planner" type="global_speed_planning" ...>
  <param name="max_speed" value="$(arg max_speed)"/>
</node>
```

따라서 실행 시 다음처럼 인자를 넘겨야 실제 런타임 파라미터가 2.0으로 설정됩니다:

```bash
roslaunch gps_global_planner motion_planning_pipeline.launch max_speed:=2.0
```

또는 실행 중에 직접 설정할 수 있지만, 노드는 시작 시 한 번만 파라미터를 읽으므로 재시작이 필요합니다:

```bash
rosparam set /global_speed_planning/max_speed 2.0
rosnode restart /global_speed_planning
```

비고: `global_speed_planning.cpp` 내부 기본값은 `pnh_.param("max_speed", max_speed_, 2.0)`이지만, 런치에서 전달되는 값이 우선합니다.