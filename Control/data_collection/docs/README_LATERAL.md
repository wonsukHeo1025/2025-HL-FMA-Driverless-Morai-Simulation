# 횡방향 제어 데이터 수집 시스템

## 개요

이 패키지는 기존 데이터 수집 시스템을 확장하여 MPC 개발을 위한 횡방향 제어 시스템 식별을 지원합니다. 차량 횡방향 동역학 특성을 파악하고 코너링 강성, 요 관성 등의 주요 파라미터를 추정하기 위한 특수 시나리오들을 포함합니다.

## 횡방향 제어 시나리오

### 1. 정상상태 선회 (`steady_state_cornering`)
- **목적**: 언더스티어 그래디언트 및 정상상태 게인 추정
- **방법**: 다양한 속도에서 일정한 조향각 유지
- **주요 파라미터**:
  - `steering_angles`: 테스트할 조향각 목록 (도)
  - `hold_duration`: 각 조향각 유지 시간 (초)
  - `target_speed`: 테스트 차량 속도 (km/h)

### 2. 스텝 조향 응답 (`step_steer`)
- **목적**: 과도 응답 분석 및 요 관성 추정
- **방법**: 조향각의 스텝 변화 적용
- **주요 파라미터**:
  - `step_angles`: 스텝 조향각 목록 (도)
  - `step_duration`: 각 스텝 유지 시간 (초)
  - `step_rise_time`: 스텝 전환 시간 (초)

### 3. 사인파 스윕 (`sine_sweep`)
- **목적**: 주파수 응답 특성 분석
- **방법**: 선형적으로 증가하는 주파수의 정현파 조향
- **주요 파라미터**:
  - `freq_start`: 시작 주파수 (Hz)
  - `freq_end`: 종료 주파수 (Hz)
  - `sweep_duration`: 전체 스윕 시간 (초)
  - `amplitude`: 조향 진폭 (도)

### 4. 더블 레인 체인지 (`double_lane_change`)
- **목적**: 실제 시나리오에서의 모델 검증
- **방법**: ISO 3888-2 스타일 차선 변경 기동
- **주요 파라미터**:
  - `lane_width`: 차선 폭 (미터)
  - `maneuver_distance`: 기동 전체 거리 (미터)

## 빠른 시작

### 패키지 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 횡방향 데이터 수집 실행

#### 정상상태 선회 (언더스티어 그래디언트 측정)
```bash
# 40 km/h에서 다양한 조향각 테스트
roslaunch data_collection lateral_data_collection.launch \
    scenario:=steady_state_cornering \
    speed:=40 \
    steering_angles:="[5, 10, 15, 20]"
```

#### 스텝 조향 응답 (과도 응답 분석)
```bash
# 40 km/h에서 스텝 조향 테스트
roslaunch data_collection lateral_data_collection.launch \
    scenario:=step_steer \
    speed:=40
```

#### 사인파 스윕 (주파수 응답)
```bash
# 0.1 Hz에서 2.0 Hz까지 주파수 스윕
roslaunch data_collection lateral_data_collection.launch \
    scenario:=sine_sweep \
    speed:=40 \
    freq_start:=0.1 \
    freq_end:=2.0 \
    sweep_duration:=60
```

#### 더블 레인 체인지 (검증용)
```bash
# 50 km/h에서 ISO 3888-2 스타일 기동
roslaunch data_collection lateral_data_collection.launch \
    scenario:=double_lane_change \
    speed:=50
```

### 데이터 기록과 함께 실행
```bash
# 오프라인 분석을 위한 bag 파일 기록
roslaunch data_collection lateral_data_collection.launch \
    scenario:=step_steer \
    speed:=40 \
    record:=true
```

### 실시간 시각화와 함께 실행
```bash
# PlotJuggler와 함께 실행
roslaunch data_collection lateral_data_collection.launch \
    scenario:=sine_sweep \
    speed:=40 \
    plot:=true
```

## Data Output

### ROS Topics

#### Published Topics
- `/lateral_system_data` (`control_msgs/LateralSystemData`): Comprehensive lateral dynamics data
- `/ctrl_cmd` (`morai_msgs/CtrlCmd`): Control commands to MORAI

#### Subscribed Topics
- `/Competition_topic` (`morai_msgs/EgoVehicleStatus`): Vehicle state from MORAI
- `/imu` (`sensor_msgs/Imu`): IMU data for yaw rate
- `/gps` (`morai_msgs/GPSMessage`): GPS position
- `/tf`: Transform tree for pose estimation

### CSV Data Files
Data is saved to: `~/catkin_ws/src/Control/data_collection/data/lateral/`

File naming: `{scenario}_{speed}kmh_{YYYYMMDD}_{HHMMSS}.csv`

#### CSV Columns Include:
- Time stamps
- Steering angle (rad, deg)
- Vehicle speed (m/s, km/h)
- Yaw rate (rad/s, deg/s)
- Lateral acceleration (m/s²)
- Lateral/heading errors (if path available)
- Sideslip angle
- Scenario information

## System Identification Workflow

### Phase 1: Data Collection (Current)
1. Run steady-state cornering at multiple speeds
2. Collect step steer response data
3. Perform sine sweep for frequency analysis
4. Validate with double lane change

### Phase 2: Parameter Estimation
```python
# Example parameter estimation script (to be implemented)
from lateral_system_id import estimate_parameters

# Load collected data
data = load_csv('steady_state_40kmh_20250810_143022.csv')

# Estimate understeer gradient
Kv = estimate_understeer_gradient(data)

# Estimate cornering stiffness
Caf, Car = estimate_cornering_stiffness(data, vehicle_params)

# Estimate yaw inertia from step response
Iz = estimate_yaw_inertia(step_response_data)
```

### Phase 3: Model Validation
- Compare model predictions with measured data
- Validate frequency response
- Check performance in lane change scenarios

## Model Parameters to Identify

### Kinematic Model
- `L`: Wheelbase (from vehicle specs)

### Dynamic Model
- `m`: Vehicle mass
- `Iz`: Yaw moment of inertia
- `lf`, `lr`: Front/rear axle distances from CG
- `Caf`, `Car`: Front/rear cornering stiffness
- `Kv`: Understeer gradient

## Tips for Good Data Collection

### For Steady-State Tests
- Ensure vehicle reaches steady state before recording
- Maintain constant speed throughout
- Use sufficient hold duration (10+ seconds)
- Test both positive and negative steering angles

### For Step Steer Tests
- Start from straight driving
- Apply step as quickly as possible (<0.2s)
- Hold for sufficient time to observe settling
- Include recovery period between steps

### For Sine Sweep
- Start with low frequency (0.1 Hz)
- Increase frequency gradually
- Use moderate amplitude (5-10 degrees)
- Ensure sufficient duration for each frequency

### For Double Lane Change
- Maintain constant speed
- Follow ISO 3888-2 guidelines if applicable
- Record multiple runs for consistency

## Troubleshooting

### Issue: Noisy yaw rate measurements
**Solution**: Check IMU mounting and calibration. Apply low-pass filter if needed.

### Issue: Inconsistent steady-state results
**Solution**: Increase hold duration, ensure speed control is stable.

### Issue: Poor frequency response at high frequencies
**Solution**: Increase control rate (>50Hz), reduce system delays.

### Issue: Large path tracking errors
**Solution**: Verify TF tree, check GPS/IMU fusion, validate reference path.

## Integration with MPC Development

The collected data will be used in subsequent phases:

1. **Phase 2**: Offline system identification
   - Process collected CSV files
   - Estimate model parameters
   - Validate model accuracy

2. **Phase 3**: MPC design
   - Use identified parameters in MPC model
   - Design and tune controller offline
   - Simulate with collected scenarios

3. **Phase 4**: ROS integration
   - Deploy MPC controller as ROS node
   - Compare with Pure Pursuit baseline
   - Validate improvements in understeer handling

## References

- ISO 3888-2: Passenger cars — Test track for a severe lane-change manoeuvre
- Rajamani, R. (2011). Vehicle dynamics and control
- Kong, J. et al. (2015). Kinematic and dynamic vehicle models for autonomous driving

## Contact

For questions or issues, please refer to the main Control package documentation or contact the development team.

## Lateral System Identification Q&A

### 1. 조향각 테스트 시나리오 구성에 대한 답변

**Q: 최대 조향각이 40도까지인데 5, 10, 15, 20도로 테스트하는 것이 괜찮은가? 각 종방향 속도별로 테스트해야 하는가?**

A: 조향각 테스트 설정에 대한 고려사항:

**조향각 범위 설정:**
- 5, 10, 15, 20도는 기본적으로 적절한 선택입니다
- 하지만 다음을 추가로 고려해야 합니다:
  - **양방향 테스트**: -20, -15, -10, -5, 0, 5, 10, 15, 20도로 양방향 모두 테스트
  - **비선형 영역 탐색**: 25, 30도도 추가하여 타이어 슬립이 발생하는 비선형 영역 데이터 확보
  - **세밀한 간격**: 저속에서는 2.5도 간격으로 더 세밀하게 테스트 가능

**속도별 테스트 필요성:**
- **필수적으로 여러 속도에서 테스트해야 합니다**
- 추천 속도 구간:
  - 저속: 20 km/h (5.56 m/s) - 기구학 모델이 유효한 영역
  - 중속: 40 km/h (11.11 m/s) - 주 운행 속도
  - 고속: 60 km/h (16.67 m/s) - 동역학 효과가 두드러지는 영역
- 각 속도에서 타이어 슬립각과 코너링 강성의 관계가 달라지므로 속도별 데이터가 필수

**슬립 측정 방법:**
- **측정할 물리량**:
  - 전/후륜 슬립각: α_f = δ - atan((v_y + l_f * ψ_dot) / v_x)
  - 횡방향 속도 v_y: IMU 적분 또는 상태 추정기 사용
  - 횡방향 가속도 a_y: IMU 직접 측정
- **파라미터 추정**:
  - 정상상태에서 횡력 F_y = m * a_y와 슬립각 α의 관계로부터 코너링 강성 C_α 추정
  - 언더스티어 그래디언트 K_v = (m/L) * (l_r/C_αf - l_f/C_αr)

### 2. 시나리오 모드 vs 수동 rostopic pub의 필요성

**Q: rostopic pub으로 직접 제어 가능한데 굳이 시나리오 모드가 필요한가?**

A: 시나리오 모드가 필수적인 이유:

**재현성과 일관성:**
- 수동 명령은 타이밍과 전환이 불규칙하여 재현 불가
- 시나리오는 정확한 타이밍과 부드러운 전환 보장
- 여러 실험 간 일관된 비교 가능

**데이터 품질:**
- 시나리오별로 최적화된 전환 프로파일 (예: step은 0.2초, steady-state는 2초)
- 시스템 식별에 필요한 충분한 여기(excitation) 보장
- 과도 응답과 정상 상태를 명확히 구분

**자동화와 효율성:**
- 다양한 조건에서 반복 실험 자동화
- 실험 메타데이터 자동 기록
- 실시간 진행 상황 모니터링

**안전성:**
- 점진적 전환으로 차량 불안정 방지
- 제한값 자동 적용 (최대 40도)
- 비상 정지 로직 포함 가능

### 3. Sine Sweep 속도별 응답 차이

**Q: Sine sweep에서 속도별로 반응이 다를 수 있는데 괜찮은가?**

A: 맞습니다. 속도별로 다른 주파수 응답을 보이는 것이 정상이며, 오히려 중요한 정보입니다:

**속도별 테스트 전략:**
- **저속 (20 km/h)**: 0.1-1.0 Hz 범위, 기구학적 거동 우세
- **중속 (40 km/h)**: 0.1-2.0 Hz 범위, 과도기적 거동
- **고속 (60 km/h)**: 0.1-3.0 Hz 범위, 동역학적 거동 우세

**속도가 주파수 응답에 미치는 영향:**
- 고유 주파수 변화: ω_n = sqrt(C_α * L / (m * v_x))
- 감쇠비 변화: ζ = f(v_x)
- 위상 지연 증가: 고속일수록 큰 위상 지연

**데이터 활용:**
- 각 속도에서의 전달함수 도출
- 속도 의존적 파라미터 맵핑
- 게인 스케줄링 MPC 설계 기초 자료

### 4. 모델 파라미터와 측정 물리량

**Q: 구하려는 모델이 무엇이고, 파라미터가 무엇이며, 어떤 물리량이 필요한가?**

A: Dynamic Bicycle Model 기반 시스템 식별:

**목표 모델: Dynamic Bicycle Model**
```
상태: x = [y, v_y, ψ, ψ_dot]'
입력: u = [δ_f]
외란: d = [곡률 κ]
```

**추정할 파라미터:**
1. **차량 파라미터** (제원에서 확인 가능):
   - m: 차량 질량 (kg)
   - L: 휠베이스 (m)
   - l_f, l_r: 전/후륜 무게중심 거리 (m)

2. **동역학 파라미터** (추정 필요):
   - I_z: 요 관성 모멘트 (kg·m²)
   - C_αf: 전륜 코너링 강성 (N/rad)
   - C_αr: 후륜 코너링 강성 (N/rad)
   - K_v: 언더스티어 그래디언트 (rad/(m/s²))

**필요한 측정 물리량:**

| 물리량 | 측정 방법 | 용도 |
|--------|-----------|------|
| 조향각 δ | /ctrl_cmd topic | 입력 신호 |
| 종방향 속도 v_x | /Competition_topic | 모델 파라미터 |
| 횡방향 속도 v_y | 상태 추정 또는 적분 | 상태 변수 |
| 요 레이트 ψ_dot | IMU (/imu topic) | 주요 출력 |
| 횡방향 가속도 a_y | IMU | 검증용 |
| 위치 (x, y) | GPS + TF | 경로 추종 오차 |

**시나리오별 파라미터 추정 방법:**

1. **Steady-State Cornering → K_v, 정상상태 게인**
   - 방법: δ vs a_y 그래프의 기울기
   - 최소자승법으로 선형 회귀
   - K_v = Δδ/Δa_y

2. **Step Steer → I_z, 과도 응답 특성**
   - 방법: 요 레이트 시간 응답 분석
   - Rise time, settling time으로부터 I_z 추정
   - 시스템 식별 도구 사용 (MATLAB System Identification Toolbox 또는 Python scipy)

3. **Sine Sweep → C_αf, C_αr, 전체 검증**
   - 방법: 주파수 응답 함수 (FRF) 분석
   - Bode plot으로부터 파라미터 추정
   - 비선형 최적화로 정밀 조정

**데이터 처리 차이점:**
- **종방향 (기존)**: 단순 최소자승법으로 충분 (선형 관계)
- **횡방향 (현재)**: 
  - 비선형 최적화 필요 (여러 파라미터 동시 추정)
  - 주파수 영역 분석 병행
  - 모델 검증 단계 필수
  - Grey-box 시스템 식별 기법 활용

**구현 예시 (Python):**
```python
from scipy.optimize import least_squares
import numpy as np

def dynamic_bicycle_model(params, inputs, dt):
    """Dynamic bicycle model for parameter estimation"""
    Iz, Caf, Car = params
    # ... 모델 구현 ...
    return predicted_states

def cost_function(params, measured_data, inputs, dt):
    """Cost function for parameter optimization"""
    predicted = dynamic_bicycle_model(params, inputs, dt)
    error = measured_data - predicted
    return error.flatten()

# 파라미터 추정
initial_guess = [Iz_init, Caf_init, Car_init]
result = least_squares(cost_function, initial_guess, 
                       args=(measured_data, inputs, dt))
Iz_est, Caf_est, Car_est = result.x
```

이러한 체계적 접근을 통해 Pure Pursuit의 언더스티어 문제를 개선하는 정확한 MPC 모델을 구축할 수 있습니다.

### 5. 코너링 강성(Cornering Stiffness)의 실제 특성

**Q: 코너링 강성이 시시각각 변하는데 상수로 취급해도 되는가? 시뮬레이터에서 직진 시 0, 코너링 시 10000 N/deg까지 표시되는 이유는?**

A: 코너링 강성의 비선형 특성과 모델링 전략:

**코너링 강성은 실제로 상수가 아닙니다:**

1. **비선형 특성**
   - **선형 영역**: 슬립각 < 3-5도에서만 근사적으로 선형
   - **과도 영역**: 5-10도에서 강성이 점진적으로 감소
   - **포화 영역**: 10도 이상에서 타이어 포화, 강성 급격히 감소

2. **영향 요인**
   - **수직 하중 Fz**: 하중 증가 시 강성 증가 (비선형 관계)
   - **속도**: 고속에서 강성 변화
   - **노면 마찰계수**: 젖은 노면에서 감소
   - **타이어 온도**: 온도에 따른 고무 특성 변화

**시뮬레이터 표시값 해석:**

```
직진 시: 0 N/deg
코너링 시: ~10000 N/deg
```

이는 다음 중 하나입니다:

1. **순간 유효 강성 (Effective Stiffness)**
   - C_α_eff = F_y / α (현재 횡력 / 현재 슬립각)
   - 직진 시: α = 0 → 0/0 = undefined → 0으로 표시
   - 실제 초기 강성(dF_y/dα at α=0)은 0이 아님

2. **작동점 강성 (Operating Point Stiffness)**
   - 현재 슬립각에서의 국소 기울기
   - 타이어 곡선의 접선 기울기

**단위 변환:**
- 10000 N/deg = 573000 N/rad
- 일반적인 승용차 전륜: 80000-120000 N/rad
- 후륜: 100000-150000 N/rad

**MPC 모델링 전략:**

**Level 1: 선형 상수 모델 (초기 구현)**
```python
# 평균적인 작동점에서의 상수값 사용
C_αf = 100000  # N/rad (고정)
C_αr = 120000  # N/rad (고정)
# 장점: 단순, MPC 계산 빠름
# 단점: 큰 슬립각에서 부정확
```

**Level 2: 구간별 선형화 (중급)**
```python
def get_cornering_stiffness(slip_angle):
    α_deg = np.degrees(slip_angle)
    if abs(α_deg) < 3:
        return 100000  # 선형 영역
    elif abs(α_deg) < 7:
        return 80000   # 과도 영역  
    else:
        return 50000   # 포화 영역
```

**Level 3: 비선형 타이어 모델 (고급)**
```python
# Pacejka Magic Formula
def pacejka_tire_model(α, Fz):
    # B: Stiffness factor
    # C: Shape factor  
    # D: Peak value
    # E: Curvature factor
    B, C, D, E = get_coefficients(Fz)
    F_y = D * sin(C * atan(B*α - E*(B*α - atan(B*α))))
    
    # 순간 강성 (미분값)
    C_α = dF_y_dα(α, Fz)
    return F_y, C_α

# Fiala 모델 (더 단순)
def fiala_model(α, C_α0, α_sl):
    if abs(α) < α_sl/3:
        F_y = C_α0 * α  # 선형
    elif abs(α) < α_sl:
        F_y = C_α0 * α_sl * (1 - (α/α_sl - 1)**3) * sign(α)  # 과도
    else:
        F_y = C_α0 * α_sl * sign(α)  # 포화
    return F_y
```

**Level 4: 적응형 MPC (최종 목표)**
```python
class AdaptiveMPC:
    def __init__(self):
        self.C_α_estimator = ExtendedKalmanFilter()
        
    def update(self, measured_data):
        # 실시간 파라미터 추정
        C_αf_est, C_αr_est = self.C_α_estimator.estimate(
            measured_data['F_y'], 
            measured_data['α'],
            measured_data['Fz']
        )
        
        # MPC 모델 업데이트
        self.update_model(C_αf_est, C_αr_est)
```

**실용적 접근법:**

1. **Phase 1 (현재)**: 
   - 데이터 수집 시 다양한 슬립각 조건 포함
   - 평균적인 선형 C_α로 시작

2. **Phase 2**: 
   - 수집 데이터로 슬립각-횡력 맵 구축
   - Lookup table 또는 다항식 피팅

3. **Phase 3**: 
   - 비선형 MPC 또는 Gain-scheduled MPC
   - 슬립각 범위별로 다른 선형 모델 사용

**핵심 포인트:**
- MPC 설계 시 "평균적인" C_α를 초기값으로 사용
- 실제 주행 조건(40km/h, 작은 조향각)에서는 선형 근사로 충분
- 극한 상황 대응이 필요하면 비선형 모델 도입

### 6. 과도한 타이어 슬립 데이터의 처리

**Q: 선형 모델을 사용하는데 타이어 슬립이 과도하게 일어나는 데이터로 학습하면 어떻게 되는가?**

A: 매우 중요한 문제입니다. 비선형 영역 데이터로 선형 모델을 학습시키면 심각한 문제가 발생합니다:

**발생 가능한 문제들:**

1. **파라미터 추정 오류**
   - 비선형 영역 데이터 포함 시 추정된 C_α가 실제보다 작아짐
   - 예: 실제 선형 영역 C_α = 100000 N/rad
   - 비선형 포함 추정: C_α = 60000 N/rad (심각한 과소평가)

2. **모델 예측 성능 저하**
   ```python
   # 잘못된 학습 결과
   선형 영역 (α < 5°): 모델이 과도하게 반응 (오버슈팅)
   비선형 영역 (α > 10°): 모델이 부족하게 반응 (언더슈팅)
   ```

3. **MPC 제어 성능 악화**
   - 잘못된 모델로 인한 부정확한 예측
   - 불필요하게 큰 제어 입력 생성
   - 진동이나 불안정성 발생 가능

**데이터 필터링 전략:**

**방법 1: 슬립각 기반 필터링**
```python
def filter_linear_region_data(data):
    """선형 영역 데이터만 추출"""
    # 전륜 슬립각 계산
    alpha_f = data['delta'] - np.arctan2(
        data['v_y'] + data['l_f'] * data['yaw_rate'], 
        data['v_x']
    )
    
    # 후륜 슬립각 계산
    alpha_r = -np.arctan2(
        data['v_y'] - data['l_r'] * data['yaw_rate'],
        data['v_x']
    )
    
    # 선형 영역만 선택 (5도 이하)
    linear_mask = (np.abs(alpha_f) < np.radians(5)) & \
                  (np.abs(alpha_r) < np.radians(5))
    
    return data[linear_mask]
```

**방법 2: 횡가속도 기반 필터링**
```python
def filter_by_lateral_acceleration(data, a_y_max=4.0):
    """낮은 횡가속도 영역만 선택 (m/s²)"""
    # 일반적으로 0.4g (약 4 m/s²) 이하가 선형 영역
    linear_mask = np.abs(data['a_y']) < a_y_max
    return data[linear_mask]
```

**방법 3: 통계적 이상치 제거**
```python
def remove_outliers_ransac(data):
    """RANSAC으로 비선형 데이터 제거"""
    from sklearn.linear_model import RANSACRegressor
    
    X = data[['slip_angle']].values
    y = data['lateral_force'].values
    
    # RANSAC으로 선형 관계만 피팅
    ransac = RANSACRegressor(residual_threshold=500)  # N
    ransac.fit(X, y)
    
    # 인라이어만 반환
    inlier_mask = ransac.inlier_mask_
    return data[inlier_mask]
```

**방법 4: 구간별 가중치 적용**
```python
def weighted_least_squares(data):
    """슬립각에 따른 가중치 적용"""
    alpha = data['slip_angle']
    F_y = data['lateral_force']
    
    # 선형 영역에 높은 가중치
    weights = np.exp(-alpha**2 / (2 * np.radians(3)**2))
    
    # 가중 최소자승법
    C_alpha = np.sum(weights * F_y * alpha) / \
              np.sum(weights * alpha**2)
    
    return C_alpha
```

**실용적 데이터 수집 가이드라인:**

1. **시나리오별 조향각 제한**
   - Steady-state: 최대 15도 (선형 영역 중심)
   - Step steer: 최대 10도 스텝
   - Sine sweep: 5-7도 진폭

2. **속도별 조향각 조정**
   ```python
   def get_max_steering_angle(speed_kmh):
       """속도에 따른 최대 조향각 (도)"""
       if speed_kmh < 30:
           return 20  # 저속: 큰 각도 허용
       elif speed_kmh < 50:
           return 15  # 중속: 중간
       else:
           return 10  # 고속: 작은 각도만
   ```

3. **데이터 검증 프로세스**
   ```python
   def validate_linearity(data):
       """수집 데이터의 선형성 검증"""
       # 1. 슬립각 vs 횡력 플롯
       plt.scatter(data['slip_angle'], data['lateral_force'])
       
       # 2. 선형 피팅
       linear_fit = np.polyfit(data['slip_angle'], 
                               data['lateral_force'], 1)
       
       # 3. R² 계산
       r_squared = calculate_r2(data, linear_fit)
       
       if r_squared < 0.95:
           print("경고: 비선형 데이터 포함 가능성")
           
       return r_squared > 0.95
   ```

4. **단계별 접근법**
   - **Phase 1**: 보수적인 조향각으로 순수 선형 데이터만 수집
   - **Phase 2**: 선형 모델 파라미터 추정 및 검증
   - **Phase 3**: 필요시 비선형 영역 추가 수집 (별도 모델용)

**핵심 권장사항:**
- 선형 MPC용 데이터는 **반드시 선형 영역에서만** 수집
- 슬립각 5도 이하, 횡가속도 0.4g 이하 유지
- 데이터 수집 후 선형성 검증 필수
- 의심스러운 데이터는 과감히 제외

이렇게 하지 않으면 "쓰레기 입력, 쓰레기 출력(GIGO)" 현상으로 MPC 성능이 Pure Pursuit보다 못할 수 있습니다.