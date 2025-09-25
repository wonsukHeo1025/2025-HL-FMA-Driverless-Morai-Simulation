# MPC Longitudinal Controller

ROS 기반 Model Predictive Control (MPC) 종방향 제어기 패키지입니다. MORAI 시뮬레이터와 연동하여 차량의 속도를 제어합니다.

## 📋 개요

이 패키지는 차량의 종방향 동역학 모델을 기반으로 목표 속도를 추종하는 MPC 제어기를 구현합니다.

### 주요 특징
- CVXPY 기반 실시간 최적화 (50Hz)
- 속도 제약 및 입력 제약 처리
- Jerk 억제를 통한 부드러운 주행
- 모듈식 설계 (ROS 독립적 코어 로직)

### 시스템 파라미터
- **A = 0.9956**: 시스템 행렬 (Phase 1에서 식별)
- **B = 0.0779**: 입력 행렬
- **d = 0.0336**: 외란/오프셋

## 🚀 설치

### 의존성
```bash
# Python 패키지
pip3 install cvxpy numpy

# ROS 패키지
sudo apt-get install ros-noetic-morai-msgs
```

### 빌드
```bash
cd ~/catkin_ws
catkin_make --only-pkg-with-deps mpc_longitudinal_controller
source devel/setup.bash
```

## 🎮 실행 방법

### 기본 실행
```bash
roslaunch mpc_longitudinal_controller mpc_controller.launch
```

### 커스텀 설정으로 실행
```bash
# 다른 파라미터 파일 사용
roslaunch mpc_longitudinal_controller mpc_controller.launch config_file:=/path/to/params.yaml

# 디버그 모드 (테스트용 목표 속도 자동 발행)
roslaunch mpc_longitudinal_controller mpc_controller.launch debug:=true
```

## 📊 ROS 인터페이스

> **주의**: 모든 속도 관련 토픽은 **km/h 단위**를 사용합니다. (내부 MPC 계산은 m/s)

### 입력 토픽

| 토픽명 | 타입 | 설명 |
|--------|------|------|
| `/Competition_topic` | `morai_msgs/EgoVehicleStatus` | 현재 차량 상태 (velocity.x는 m/s로 수신, 내부에서 km/h로 변환) |
| `/target_velocity` | `std_msgs/Float64` | 목표 속도 [**km/h**] |

### 출력 토픽

| 토픽명 | 타입 | 설명 |
|--------|------|------|
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` | 차량 제어 명령 (accel/brake) |
| `~current_velocity_kmph` | `std_msgs/Float64` | 현재 속도 [**km/h**] (디버그용) |
| `~target_velocity_kmph` | `std_msgs/Float64` | 목표 속도 [**km/h**] (디버그용) |
| `~control_input` | `std_msgs/Float64` | MPC 제어 입력 (디버그용) |

## ⚙️ 파라미터 설정

`config/mpc_params.yaml` 파일에서 수정 가능:

### MPC 알고리즘 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `Np` | 20 | 예측 구간 (Prediction Horizon) |
| `Nc` | 5 | 제어 구간 (Control Horizon) |
| `Q` | 1.0 | 속도 추종 가중치 (크면 추종 강화) |
| `R` | 0.5 | 제어 입력 가중치 (크면 부드러운 제어) |
| `R_delta` | 0.1 | 입력 변화 가중치 (크면 급가감속 억제) |

### 제약 조건

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `v_min_kmph` | 0.0 | 최소 속도 [**km/h**] |
| `v_max_kmph` | 40.0 | 최대 속도 [**km/h**] |
| `u_min` | -1.0 | 최대 제동 |
| `u_max` | 1.0 | 최대 가속 |
| `delta_u_max` | 0.2 | 최대 입력 변화량 (jerk 제한) |

## 🧪 테스트 방법

### 1. 목표 속도 수동 발행
```bash
# 20 km/h로 설정
rostopic pub /target_velocity std_msgs/Float64 "data: 20.0"

# 35 km/h로 변경
rostopic pub /target_velocity std_msgs/Float64 "data: 35.0"

# 정지
rostopic pub /target_velocity std_msgs/Float64 "data: 0.0"
```

### 2. 상태 모니터링
```bash
# 제어기 상태 확인 (km/h 단위)
rostopic echo /mpc_controller/current_velocity_kmph
rostopic echo /mpc_controller/target_velocity_kmph
rostopic echo /mpc_controller/control_input

# 제어 명령 확인
rostopic echo /ctrl_cmd
```

### 3. 실시간 시각화
```bash
# PlotJuggler 실행
rosrun plotjuggler plotjuggler

# 다음 토픽들을 추가하여 그래프 확인:
# - /mpc_controller/current_velocity_kmph  (km/h)
# - /mpc_controller/target_velocity_kmph   (km/h)
# - /mpc_controller/control_input
```

## 📈 성능 튜닝 가이드

### 추종 성능 개선
- **느린 응답**: `Q` 값 증가 (예: 1.0 → 2.0)
- **오버슈트 발생**: `R` 값 증가 (예: 0.5 → 1.0)
- **진동 발생**: `R_delta` 값 증가 (예: 0.1 → 0.3)

### 주행 품질 개선
- **급가속/급제동**: `delta_u_max` 감소 (예: 0.2 → 0.1)
- **부드러운 주행**: `R` 및 `R_delta` 증가

### 계산 성능
- **실시간성 확보**: `Np` 또는 `Nc` 감소
- **예측 정확도 향상**: `Np` 증가 (단, 계산 시간 증가)

## 🔍 예상 동작

### 정상 실행 시 콘솔 출력
```
[INFO] MPC Core initialized with A=0.9956, B=0.0779, d=0.0336
[INFO] MPC Controller Node initialized and running
[INFO]   Control rate: 50.0 Hz
[INFO]   Subscribed to: /Competition_topic, /target_velocity
[INFO]   Publishing to: /ctrl_cmd
[INFO]   Speed units: External interface uses km/h, internal MPC uses m/s
[INFO] Velocity: 0.0 km/h, Target: 20.0 km/h, Control: 0.456 (A:0.46, B:0.00)
```

### 성능 지표
- **정착 시간**: 2-3초 이내
- **오버슈트**: 10% 이내
- **정상상태 오차**: 0.5 km/h 이내
- **제어 주기**: 20ms (50Hz)

## 🐛 문제 해결

### CVXPY 관련 오류
```bash
# CVXPY 재설치
pip3 install --upgrade cvxpy

# OSQP 솔버 설치
pip3 install osqp
```

### 속도 업데이트 없음 경고
- `/Competition_topic` 토픽이 발행되고 있는지 확인
- `timeout_duration` 파라미터 조정 (기본 1.0초)

### 제어 불안정
1. 시스템 모델 파라미터 (A, B, d) 확인
2. `R` 및 `R_delta` 값 증가
3. `delta_u_max` 값 감소

## 📂 패키지 구조

```
mpc_longitudinal_controller/
├── config/
│   └── mpc_params.yaml         # MPC 파라미터 설정
├── launch/
│   └── mpc_controller.launch   # 실행 파일
├── scripts/
│   └── mpc_node.py            # ROS 노드 (실행 파일)
├── src/
│   └── mpc_longitudinal_controller/
│       ├── __init__.py
│       └── mpc_core.py        # MPC 핵심 알고리즘
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 📚 참고 문서

- [Phase 1: 시스템 모델링](../docs/PRD_Phase1.md)
- [Phase 2: MPC 제어기 설계](../docs/PRD_Phase2.md)
- [Phase 3: ROS 노드 구현](../docs/PRD_Phase3.md)
- [Phase 4: 검증 및 튜닝](../docs/PRD_Phase4.md)

## 📝 라이센스

TODO

## 👥 기여자

- MPC 알고리즘 설계 및 구현
- MORAI 시뮬레이터 연동