# **Phase 1: Lateral MPC를 위한 시스템 식별 및 모델 개발**

## **1. 개요 및 목표**

본 문서는 Lateral MPC(Model Predictive Control) 제어기 개발을 위한 첫 번째 단계로, **차량의 횡방향 동역학 모델의 파라미터를 식별**하고 **모델 검증**을 수행하는 상세 계획을 정의합니다.

Pure Pursuit 제어기의 언더스티어 문제를 개선하기 위해, 실제 차량의 횡방향 거동을 정확히 모델링하는 것이 핵심입니다. 본 문서는 두 가지 모델링 접근법(Kinematic/Dynamic)에 대한 데이터 수집 및 파라미터 식별 방법론을 제시합니다.

**핵심 목표**:
1. **모델 선택**: Kinematic Bicycle Model과 Dynamic Bicycle Model 중 프로젝트 요구사항에 적합한 모델 선택
2. **데이터 수집**: 조향 입력과 차량의 횡방향 응답 사이의 관계를 파악하기 위한 체계적인 데이터 수집
3. **파라미터 식별**: 수집된 데이터를 바탕으로 모델 파라미터 추정 ($L$, $C_{\alpha f}$, $C_{\alpha r}$, $I_z$ 등)
4. **모델 검증**: 식별된 파라미터의 정확성과 모델의 예측 성능 검증

## **2. 모델 선택 전략**

### **2.1. 빠른 프로토타이핑 전략 (Kinematic Model 우선)**

초기 구현의 단순성과 빠른 개발을 위해 Kinematic Model부터 시작하는 2단계 접근법:

```text
[Step 1: Kinematic Model]
  ├─ 파라미터: 휠베이스(L)만 필요
  ├─ 장점: 구현 단순, 빠른 프로토타이핑
  └─ 목표: 전체 시스템 파이프라인 구축

[Step 2: Dynamic Model 업그레이드]
  ├─ 파라미터: m, I_z, lf, lr, C_αf, C_αr
  ├─ 장점: 언더스티어/오버스티어 모델링
  └─ 목표: Pure Pursuit 대비 성능 개선
```

### **2.2. 모델별 필요 파라미터**

| 모델 | 필요 파라미터 | 식별 난이도 | 적용 속도 범위 |
|------|--------------|------------|---------------|
| Kinematic | L (휠베이스) | 매우 쉬움 | ~40 km/h |
| Dynamic | m, I_z, lf, lr, C_αf, C_αr | 어려움 | 전체 속도 범위 |

## **3. 데이터 수집 시스템 아키텍처**

### **3.1. 시스템 구성**

```text
                        +---------------------------+
[조향 시나리오] -------> | lateral_data_collection   | -------> [/ctrl_cmd]
                        |        _node.py            |
[센서 데이터] ---------> | - IMU (yaw rate)          | -------> [/lateral_system_data]
                        | - GPS (position)           |
                        | - TF (heading)             | -------> [lateral_data.csv]
                        +---------------------------+
```

### **3.2. Custom Message 정의**

```msg
# Control/control_msgs/msg/LateralSystemData.msg

Header header

# --- 1. Control Inputs ---
float64 steering_angle      # [rad] 조향각 입력
float64 vehicle_speed       # [m/s] 현재 차속

# --- 2. Raw Sensor Data ---
sensor_msgs/Imu imu_data    # Yaw rate 측정용
morai_msgs/GPSMessage gps_data

# --- 3. Measured States ---
float64 yaw_rate           # [rad/s] 측정된 yaw rate
float64 lateral_velocity   # [m/s] 추정된 횡방향 속도
float64 heading            # [rad] 차량 heading
float64 sideslip_angle     # [rad] 추정된 sideslip angle

# --- 4. Path Tracking Errors (if applicable) ---
float64 lateral_error      # [m] 경로로부터의 횡방향 오차
float64 heading_error      # [rad] 경로 접선과의 각도 오차

# --- 5. Experiment Info ---
string scenario_type       # step_steer, steady_state, sine_sweep
float64 experiment_time    # [s] 실험 시작 후 경과 시간
```

## **4. 데이터 수집 시나리오**

### **4.1. Steady-State Cornering (정상상태 선회)**

- **목적**: 언더스티어 그래디언트($K_v$) 추정, 정상상태 게인 확인
- **절차**:
  1. 일정 속도(20, 30, 40, 50 km/h) 유지
  2. 각 속도에서 다양한 조향각(5°, 10°, 15°, 20°) 적용
  3. 원형 경로 안정화 후 10초간 데이터 수집
- **수집 데이터**: 조향각, 횡가속도, yaw rate, 선회 반경

### **4.2. Step Steer Response (스텝 조향 응답)**

- **목적**: 과도응답 특성 파악, 요 관성 모멘트($I_z$) 추정
- **절차**:
  1. 직진 주행 상태에서 시작 (40 km/h)
  2. 0.2초 내에 목표 조향각(10°)까지 스텝 입력
  3. 5초간 유지하며 과도응답 기록
  4. 다양한 조향각과 속도에서 반복
- **수집 데이터**: 시간에 따른 yaw rate, lateral acceleration 변화

### **4.3. Sine Sweep (정현파 주파수 스윕)**

- **목적**: 주파수 응답 특성 분석, 동적 파라미터 정밀 추정
- **절차**:
  1. 일정 속도 유지 (40 km/h)
  2. 조향각을 정현파로 입력: $\delta(t) = A \sin(2\pi f(t) \cdot t)$
  3. 주파수를 0.1 Hz에서 2.0 Hz까지 60초간 선형 증가
  4. 진폭 A = 5°, 10°로 실험
- **수집 데이터**: 주파수별 yaw rate 게인 및 위상 지연

### **4.4. Double Lane Change (이중 차선 변경)**

- **목적**: 실제 주행 시나리오에서의 모델 검증
- **절차**:
  1. ISO 3888-2 표준 경로 설정
  2. 다양한 속도(30, 40, 50 km/h)로 주행
  3. 경로 추종 오차 및 차량 거동 기록
- **수집 데이터**: 경로 추종 오차, 최대 yaw rate, 조향각 프로파일

## **5. 파라미터 식별 방법론**

### **5.1. Kinematic Model 파라미터 (L)**

```python
# 휠베이스는 차량 제원에서 직접 확인
L = 2.875  # [m] MORAI Ioniq5 기준
```

### **5.2. Dynamic Model 파라미터 식별**

#### **Step 1: 언더스티어 그래디언트 분석**

```python
import numpy as np
from scipy import optimize

def identify_understeer_gradient(steady_state_data):
    """
    정상상태 선회 데이터로부터 언더스티어 그래디언트 추정
    """
    # 데이터 추출
    ay = steady_state_data['lateral_acceleration']  # [m/s²]
    delta = steady_state_data['steering_angle']     # [rad]
    vx = steady_state_data['vehicle_speed']          # [m/s]
    
    # 선형 회귀: delta = L/R + Kv * vx²/R
    # where ay = vx²/R, so: delta = L/vx² * ay + Kv * ay
    A = np.vstack([ay/vx**2, ay]).T
    b = delta
    
    params, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)
    L_estimated = params[0]
    Kv = params[1]
    
    return L_estimated, Kv
```

#### **Step 2: 시스템 식별을 통한 동적 파라미터 추정**

```python
def identify_dynamic_parameters(step_response_data):
    """
    과도응답 데이터를 이용한 Iz, Cαf, Cαr 추정
    """
    def model_response(params, t, delta_input, vx):
        Iz, Caf, Car = params
        # Dynamic bicycle model 시뮬레이션
        # ... (상태공간 모델 적분)
        return predicted_yaw_rate
    
    def cost_function(params):
        predicted = model_response(params, time, steering, speed)
        measured = step_response_data['yaw_rate']
        return np.sum((predicted - measured)**2)
    
    # 최적화를 통한 파라미터 추정
    initial_guess = [1500, 100000, 120000]  # Iz, Caf, Car
    result = optimize.minimize(cost_function, initial_guess,
                               bounds=[(1000, 3000), (50000, 200000), (50000, 200000)])
    
    return result.x
```

## **6. 모델 검증 절차**

### **6.1. 예측 정확도 검증**

1. **홀드아웃 데이터셋**: 수집된 데이터의 20%를 검증용으로 보관
2. **예측 vs 실측 비교**:
   - Yaw rate 예측 RMSE < 2 deg/s
   - Lateral position 예측 RMSE < 0.2 m (5초 예측 구간)
3. **주파수 응답 검증**: Bode plot 비교

### **6.2. 언더스티어 특성 검증**

```python
def validate_understeer_characteristic(model_params, test_data):
    """
    모델의 언더스티어 예측과 실제 차량 거동 비교
    """
    # 정상상태 조향각 계산
    delta_model = calculate_steady_state_steering(model_params, test_data)
    delta_actual = test_data['steering_angle']
    
    # Ackermann 조향각 (neutral steer 기준)
    delta_ackermann = L / test_data['turning_radius']
    
    # 언더스티어 그래디언트 비교
    understeer_model = (delta_model - delta_ackermann) / test_data['lateral_acceleration']
    understeer_actual = (delta_actual - delta_ackermann) / test_data['lateral_acceleration']
    
    return np.mean(np.abs(understeer_model - understeer_actual))
```

## **7. 구현 계획**

### **7.1. 패키지 구조**

```
Control/
├── lateral_mpc/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── lateral_data_collection.launch
│   ├── scripts/
│   │   └── lateral_data_collection_node.py
│   ├── src/
│   │   └── lateral_mpc/
│   │       ├── __init__.py
│   │       ├── scenarios/
│   │       │   ├── steady_state.py
│   │       │   ├── step_steer.py
│   │       │   └── sine_sweep.py
│   │       ├── identification/
│   │       │   ├── kinematic_model.py
│   │       │   └── dynamic_model.py
│   │       └── validation/
│   │           └── model_validator.py
│   └── data/
│       └── lateral/
```

### **7.2. 개발 우선순위**

1. **Week 1**: Kinematic Model 데이터 수집 시스템 구현
2. **Week 2**: Steady-State 및 Step Steer 시나리오 구현
3. **Week 3**: 파라미터 식별 알고리즘 개발
4. **Week 4**: 모델 검증 및 Dynamic Model 업그레이드 결정

## **8. 성공 지표**

| 지표 | 목표값 | 측정 방법 |
|------|--------|----------|
| 모델 예측 정확도 | Yaw rate RMSE < 2°/s | 검증 데이터셋 |
| 파라미터 수렴성 | 변동계수 < 5% | 반복 실험 |
| 언더스티어 예측 | 오차 < 10% | 정상상태 선회 |
| 실시간성 | 예측 시간 < 2ms | 프로파일링 |

## **9. 다음 단계**

본 Phase 1에서 식별된 모델과 파라미터는 Phase 2의 MPC 제어기 설계의 기반이 됩니다. 특히:
- Kinematic Model: 빠른 프로토타이핑 및 기본 성능 확보
- Dynamic Model: Pure Pursuit 대비 개선된 성능 달성
- 검증된 파라미터: MPC 예측 정확도 보장