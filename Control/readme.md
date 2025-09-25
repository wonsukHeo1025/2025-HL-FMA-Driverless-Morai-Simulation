# Control Layer - MPC-Based Autonomous Vehicle Control

## 개요
고성능 MPC(Model Predictive Control) 기반 자율주행 차량 제어 시스템. 실시간 성능을 위한 OSQP 최적화와 시스템 식별을 통한 정확한 차량 동역학 모델 적용.

## 핵심 특징
- **실시간 MPC**: OSQP 직접 구현으로 5ms 이하 solving time 달성
- **시스템 식별**: 폐루프 데이터 기반 정확한 차량 파라미터 획득
- **다중 제어 모드**: Pure Pursuit, Lateral MPC + Longitudinal MPC 유연한 전환
- **50Hz 제어 주기**: 고주파수 실시간 제어

## 패키지 구조

```
Control/
├── mpc_lateral_controller/       # 횡방향 MPC 제어 (Dynamic/Kinematic Model)
│   ├── scripts/
│   │   ├── lateral_mpc_controller_node.py  # ROS 노드
│   │   ├── path_visualizer.py              # 경로 시각화
│   │   ├── performance_monitor.py          # 성능 모니터링
│   │   └── startup_notifier.py             # 시작 알림
│   ├── src/mpc_lateral_controller/
│   │   ├── lateral_mpc_core.py             # MPC 핵심 알고리즘
│   │   ├── osqp_backend_dynamic.py         # OSQP 직접 구현 (120-200x 성능 향상)
│   │   ├── path_processor.py               # 곡률 계산 및 처리
│   │   └── state_estimator.py              # 다중 센서 융합 상태 추정
│   ├── config/
│   │   └── mpc_params.yaml                 # MPC 파라미터 설정
│   └── launch/
│       ├── dynamic_integrated_control.launch    # Dynamic 모델 통합 제어
│       └── kinematic_integrated_control.launch  # Kinematic 모델 통합 제어
│
├── mpc_longitudinal_controller/  # 종방향 MPC 제어
│   ├── scripts/
│   │   ├── mpc_node.py                     # ROS 노드
│   │   └── speed_profile_adapter.py        # 속도 프로파일 변환
│   ├── src/mpc_longitudinal_controller/
│   │   └── mpc_core.py                     # MPC 핵심 알고리즘
│   └── config/
│       └── mpc_params.yaml                 # MPC 파라미터 설정
│
├── data_collection/              # 시스템 식별용 데이터 수집
│   ├── scripts/
│   │   └── data_collection_node.py         # 메인 ROS 노드
│   ├── src/data_collection/
│   │   ├── node/
│   │   │   └── node_core.py                # ROS 노드 핵심
│   │   ├── scenario/                       # 10가지 테스트 시나리오
│   │   │   ├── step_steer.py
│   │   │   ├── sine_sweep.py
│   │   │   ├── steady_state_cornering.py
│   │   │   └── ...
│   │   ├── logging/
│   │   │   └── csv_logger.py               # CSV 데이터 로깅
│   │   └── utils/
│   │       └── signal_generator.py         # 신호 생성 유틸리티
│   └── data/                                # 수집된 CSV 데이터
│
├── lateral_system_identification/ # 시스템 파라미터 식별
│   ├── integrated_system_identifier.py     # 통합 식별 엔진
│   ├── kinematic_model_id.py               # Kinematic 모델 식별
│   ├── dynamic_model_id.py                 # Dynamic 모델 식별
│   ├── identified_parameters.yaml          # 식별된 파라미터
│   └── run_identification.sh               # 식별 실행 스크립트
│
├── pure_pursuit/                 # Pure Pursuit 제어 (C++)
│   └── src/
│       └── pure_pursuit_node.cpp           # 기본 경로 추종 제어
│
├── offline_analysis/             # 오프라인 분석 도구
│   └── offline_analysis/
│       ├── estimator.py                    # 파라미터 추정
│       └── multi_run.py                    # 다중 실행 분석
│
└── docs/                         # 상세 문서
    ├── PRD_Lateral_MPC.md                  # 횡방향 MPC 설계
    ├── PRD_Lateral_System_Identification.md # 시스템 식별 프로세스
    ├── PRD_Longitudinal_MPC.md             # 종방향 MPC 설계
    └── input_topic_structure.md            # 토픽 구조 정의
```

## 식별된 차량 파라미터

### Dynamic Bicycle Model Parameters
```yaml
# 차량 물리 파라미터 (실제 데이터 기반 식별)
mass: 1901.0            # kg (차량 질량)
yaw_inertia: 2456.54    # kg⋅m² (yaw 관성 모멘트)
wheelbase: 3.0          # m (휠베이스)
lf: 1.7                 # m (CG에서 전륜까지 거리)
lr: 1.3                 # m (CG에서 후륜까지 거리)

# 타이어 특성 (코너링 강성)
Caf: 50040.6            # N/rad (전륜 코너링 강성)
Car: 198123.4           # N/rad (후륜 코너링 강성)

# 종방향 모델 (이산 시간)
A: 0.9956               # 상태 전이 행렬
B: 0.0779               # 입력 행렬
d: 0.0336               # 외란 항
```

## MPC 제어 알고리즘

### 1. 횡방향 MPC (Lateral MPC)

#### Dynamic Model (고속/정밀 제어)
**상태 공간 (Error Dynamics)**:
```
상태: x = [e_y, ė_y, e_ψ, ė_ψ]ᵀ
- e_y: 횡방향 오차 [m]
- ė_y: 횡방향 오차 변화율 [m/s]
- e_ψ: 헤딩 오차 [rad]
- ė_ψ: 헤딩 오차 변화율 [rad/s]

입력: u = δ (조향각) [rad]
```

**최적화 문제**:
```
min J = Σ(xᵀQx + uᵀRu + ΔuᵀR_Δu)
s.t. |δ| ≤ 40°, |Δδ| ≤ 5°/step
```

#### OSQP Backend 최적화
- **성능**: CVXPY 대비 120-200x 속도 향상
- **Warm-start**: 이전 솔루션 활용으로 수렴 속도 향상
- **Sparse Matrix**: 효율적인 메모리 사용

### 2. 종방향 MPC (Longitudinal MPC)

**시스템 모델**:
```
x[k+1] = A⋅x[k] + B⋅u[k] + d
A = 0.9956, B = 0.0779, d = 0.0336
```

**제약 조건**:
- 속도: -20 ~ 40 km/h
- 제어 입력: -1 ~ 1
- 입력 변화율: ±0.2/step (저크 억제)

## 시스템 식별 워크플로우

```
1. 데이터 수집 (data_collection)
   ├── 10가지 시나리오 실행
   ├── step_steer, sine_sweep, steady_state_cornering 등
   └── 20+ CSV 파일 생성

2. 파라미터 식별 (lateral_system_identification)
   ├── 신호 전처리 및 필터링
   ├── 모델 피팅 (최소자승법)
   └── 검증 및 평가

3. MPC 적용 (mpc_*_controller)
   ├── identified_parameters.yaml 로드
   └── 실시간 제어 수행
```

## 제어 모드 통합

### 모드 1: Pure Pursuit 단독
```
경로 추종 + 속도 제어 모두 수행
간단하지만 고속에서 성능 제한
```

### 모드 2: Pure Pursuit + MPC Longitudinal
```
Pure Pursuit: 조향각만 출력
MPC Longitudinal: 속도 제어 + 조향 통합
중간 복잡도, 안정적 성능
```

### 모드 3: Lateral MPC + MPC Longitudinal (권장)
```
Lateral MPC: Dynamic 모델 기반 정밀 조향
MPC Longitudinal: 최적 속도 제어
최고 성능, 고속 주행 적합
```

## 빌드 및 설치

### 의존성 설치
```bash
# Python 패키지
pip install cvxpy numpy scipy pandas matplotlib osqp

# ROS 패키지
sudo apt-get install ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs
```

### 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 실행 방법

### 1. 데이터 수집 (시스템 식별용)
```bash
# 기본 시나리오 실행
roslaunch data_collection data_collection.launch

# 특정 시나리오 실행
roslaunch data_collection data_collection.launch scenario:=step_steer
roslaunch data_collection data_collection.launch scenario:=sine_sweep
roslaunch data_collection data_collection.launch scenario:=steady_state_cornering

# 횡방향 데이터 수집 (완전 자동화)
roslaunch data_collection lateral_data_collection.launch
```

### 2. 시스템 파라미터 식별
```bash
cd ~/catkin_ws/src/Control/lateral_system_identification
./run_identification.sh

# 또는 Python 직접 실행
python integrated_system_identifier.py
```

### 3. MPC 제어 실행

#### Dynamic Model 통합 제어 (권장)
```bash
# Lateral MPC (Dynamic) + Longitudinal MPC
roslaunch mpc_lateral_controller dynamic_integrated_control.launch
```

#### Kinematic Model 통합 제어
```bash
# Lateral MPC (Kinematic) + Longitudinal MPC
roslaunch mpc_lateral_controller kinematic_integrated_control.launch
```

#### Pure Pursuit 단독
```bash
rosrun pure_pursuit pure_pursuit_node
```

## 성능 모니터링

### 실시간 성능 메트릭
```bash
# MPC 성능 모니터
rosrun mpc_lateral_controller performance_monitor.py

# 출력 메트릭:
# - Solve Time: < 5ms (목표)
# - Cross Track Error: < 0.1m
# - Heading Error: < 5°
# - Control Frequency: 50Hz
```

### PlotJuggler 시각화
```bash
rosrun plotjuggler plotjuggler

# 모니터링 토픽:
# - /lateral_mpc/debug: MPC 상태 및 성능
# - /lateral_mpc/timing: 솔버 타이밍
# - /lateral_mpc/status: 제어기 상태
```

### RViz 시각화
```bash
rviz -d ~/catkin_ws/src/Control/mpc_lateral_controller/rviz/mpc_visualization.rviz

# 시각화 요소:
# - 예측 경로 (초록색)
# - 참조 경로 (파란색)
# - 차량 위치 (빨간색)
# - Lookahead 포인트
```

## 파라미터 튜닝

### MPC 가중치 조정
```yaml
# mpc_lateral_controller/config/mpc_params.yaml
Q: [10.0, 1.0, 10.0, 1.0]  # 상태 가중치 [e_y, ė_y, e_ψ, ė_ψ]
R: 1.0                      # 제어 입력 가중치
R_delta: 0.1                # 입력 변화율 가중치

# 튜닝 가이드:
# - Q[0] 증가: 횡방향 오차 감소 우선
# - Q[2] 증가: 헤딩 오차 감소 우선
# - R 증가: 부드러운 조향 (제어 노력 감소)
# - R_delta 증가: 저크 억제 (승차감 개선)
```

### 예측 구간 설정
```yaml
Np: 20  # 예측 구간 (0.4초 @ 50Hz)
Nc: 5   # 제어 구간

# 가이드:
# - Np 증가: 더 먼 미래 고려 (계산 시간 증가)
# - Nc 증가: 더 정밀한 제어 (계산 시간 증가)
```

## 문제 해결

### OSQP 수렴 실패
```python
# osqp_backend_dynamic.py에서 fallback 설정
if not converged:
    # Fallback to previous solution
    return self.previous_solution
    
# 또는 파라미터 완화
solver.update_settings(eps_abs=1e-3, eps_rel=1e-3)
```

### 실시간 성능 미달
```bash
# CPU 우선순위 설정
sudo nice -n -20 roslaunch mpc_lateral_controller dynamic_integrated_control.launch

# 또는 제어 주파수 감소
roslaunch mpc_lateral_controller dynamic_integrated_control.launch control_rate:=30
```

## 성능 벤치마크

| 메트릭 | Pure Pursuit | Kinematic MPC | Dynamic MPC |
|--------|--------------|---------------|-------------|
| Solve Time | < 1ms | 3-5ms | 4-6ms |
| CTE @ 10m/s | 0.15m | 0.08m | 0.05m |
| CTE @ 20m/s | 0.35m | 0.15m | 0.08m |
| Max Speed | 15m/s | 20m/s | 30m/s |
| Stability | 중간 | 높음 | 매우 높음 |

## 참고 문서
- [횡방향 MPC 설계](docs/PRD_Lateral_MPC.md)
- [시스템 식별 프로세스](docs/PRD_Lateral_System_Identification.md)
- [종방향 MPC 설계](docs/PRD_Longitudinal_MPC.md)
- [파라미터 튜닝 가이드](mpc_lateral_controller/docs/Parameter_Tuning.md)
- [진동 완화 기법](mpc_lateral_controller/docs/lateral_mpc_oscillation_mitigation_ko.md)