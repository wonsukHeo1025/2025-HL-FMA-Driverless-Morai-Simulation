# Data Collection Package

ROS 기반 자율주행 차량 시스템 식별을 위한 데이터 수집 패키지입니다. 모듈형 아키텍처로 리팩토링되어 유지보수성과 확장성이 향상되었습니다.

## 개요

이 패키지는 MORAI 시뮬레이터 환경에서 차량의 동적 응답 데이터를 수집하여 MPC(Model Predictive Control) 설계에 필요한 시스템 모델을 구축하기 위한 도구입니다.

## 주요 기능

- 다양한 테스트 시나리오 지원 (Step Response, PRBS, Chirp)
- 실시간 센서 데이터 융합 (IMU, GPS, TF)
- CSV 형식의 구조화된 데이터 로깅
- 인터랙티브 모드와 매뉴얼 제어 지원
- 50Hz 제어 루프 실행
- 모듈형 아키텍처로 확장성 극대화

## 시스템 아키텍처

### 현재 구조 (v2 - Refactored)

```
data_collection/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── launch/
│   └── data_collection.launch     # ROS launch 파일
├── scripts/
│   └── data_collection_node.py    # 간소화된 엔트리 포인트 (50줄)
├── src/data_collection/
│   ├── scenario/                  # 시나리오 구현체
│   │   ├── base.py               # 추상 베이스 클래스
│   │   ├── step_accel.py         # Step acceleration
│   │   ├── step_brake.py         # Step brake test
│   │   ├── prbs.py               # PRBS signal
│   │   └── chirp.py              # Chirp signal
│   ├── node/                      # ROS 노드 컴포넌트
│   │   ├── interface.py          # ROS pub/sub 래퍼
│   │   ├── node_core.py          # 코어 노드 로직
│   │   └── cli_handler.py        # CLI 상호작용
│   ├── logging/                   # 데이터 로깅
│   │   └── csv_logger.py         # 범용 CSV 로거
│   └── utils/                     # 유틸리티
│       ├── filters.py            # 신호 필터링
│       └── conversions.py        # 단위 변환
└── data/                          # 수집된 데이터 저장
```

### 아키텍처 특징

#### 1. 관심사의 분리 (Separation of Concerns)
- **ROS Interface**: 모든 ROS 관련 작업이 `interface.py`에 격리
- **Business Logic**: 시나리오 로직이 ROS 의존성과 분리
- **UI/CLI**: 대화형 컴포넌트가 코어 로직과 분리
- **Data Persistence**: 메시지 타입과 독립적인 범용 로깅

#### 2. 테스트 가능성 (Testability)
- 순수 Python 모듈은 ROS 없이 테스트 가능
- 단위 테스트를 위한 Mock 친화적 인터페이스
- 명확한 의존성과 데이터 흐름

#### 3. 확장성 (Extensibility)
- 새 시나리오: `BaseScenario`를 상속한 클래스 추가
- 새 제어 모드: `ROSInterface.publish_control_command()` 수정
- 새 데이터 형식: `CSVLogger` 확장 또는 새 로거 클래스 생성

#### 4. 유지보수성 (Maintainability)
- 단일 책임 원칙: 각 모듈이 하나의 명확한 목적
- DRY 원칙: 공통 기능을 유틸리티로 추출
- 정의된 인터페이스로 명확한 모듈 경계

## 설치 및 빌드

```bash
# Catkin workspace로 이동
cd ~/catkin_ws

# 빌드
catkin_make

# 환경 설정
source devel/setup.bash
```

## 사용법

### 기본 실행

```bash
# 기본 설정으로 실행 (인터랙티브 모드)
roslaunch data_collection data_collection.launch
```

### Launch 파라미터

- `scenario`: 실행할 시나리오 타입 (기본값: 인터랙티브 선택)
- `control_rate`: 제어 루프 주파수 (기본값: 50Hz)
- `control_mode`: 제어 모드 - throttle/velocity/acceleration (기본값: throttle)
- `interactive`: 인터랙티브 모드 활성화 (기본값: true)

## 모듈 상세 설명

### Scenario Package (`scenario/`)
- **BaseScenario**: 시나리오 인터페이스를 정의하는 추상 클래스
  - `setup()`: 파라미터 초기화
  - `get_control_command(elapsed)`: 시간 t에서의 제어 명령
  - `get_total_duration()`: 예상 시나리오 길이
  - `start()`, `update()`, `is_complete()`: 생명주기 메서드

- **ScenarioFactory**: 동적 시나리오 생성
  - `create(type, params)`: 시나리오 인스턴스 생성
  - `register(name, class)`: 커스텀 시나리오 추가

### Node Package (`node/`)
- **ROSInterface**: 모든 ROS 통신 처리
  - Publishers: `/ctrl_cmd`, `/control_system_data`
  - Subscribers: `/Competition_topic`, `/imu`, `/gps`
  - TF listener for position data

- **DataCollectionNodeCore**: 메인 상태 머신
  - 상태: INIT → STABILIZING → READY → COUNTDOWN/SPEED_BUILDING → RUNNING → COMPLETE
  - 시나리오 실행 및 데이터 로깅 관리


- **CLIHandler**: 사용자 상호작용
  - 시나리오 선택 (1-4)
  - 모드 변경 (m, c)
  - 상태 표시 (i)
  - 도움말 시스템 (h)

### Utils Package (`utils/`)
- **filters.py**:
  - `LowPassFilter`: 간단한 지수 필터
  - `AdaptiveBiasEstimator`: IMU 바이어스 보상

- **conversions.py**:
  - GPS 좌표 변환
  - 단위 변환 (m/s ↔ km/h)
  - 각도 정규화
  - 쿼터니언 to 오일러

### Logging Package (`logging/`)
- **CSVLogger**: 범용 데이터 로거
  - 첫 데이터로부터 헤더 자동 생성
  - 성능을 위한 버퍼링된 쓰기
  - 타임스탬핑 및 파일 관리

## 시나리오 설명

### 1. Step Acceleration (step_accel)
- 목적: 가속 응답 특성 파악
- 동작: 0.2부터 1.0까지 0.2 단위로 가속 명령을 단계적으로 증가
- 각 스텝: 10초 가속, 5초 휴지

### 2. Step Brake (step_brake)
- 목적: 감속 응답 특성 파악
- 동작: 50km/h로 가속 후, 0.2부터 1.0까지 브레이크 테스트
- 각 스텝: 8초 브레이크, 7초 재가속
- 특징: 속도 안정화 후 테스트 시작 (2초 안정화)

### 3. PRBS (Pseudo-Random Binary Sequence)
- 목적: 시스템 동적 특성의 광대역 분석
- 동작: 0.3과 0.7 사이를 무작위로 전환하는 가속 명령
- 전환 시간: 0.5~2.0초 무작위

### 4. Chirp Signal
- 목적: 주파수 응답 특성 분석
- 동작: 0.1Hz에서 2.0Hz까지 선형적으로 증가하는 사인파 입력
- 진폭: 0.3, 오프셋: 0.4

## CSV 데이터 필드 설명

수집된 데이터는 `data/` 디렉토리에 `{scenario}_{YYYYMMDD}_{HHMMSS}.csv` 형식으로 저장됩니다.

### 시간 정보 필드

| 필드명 | 설명 | 단위 | 산출 방법 |
|--------|------|------|-----------|
| `timestamp` | ROS 시스템 타임스탬프 | seconds | `rospy.Time.now().to_sec()` |
| `scenario_time` | 시나리오 시작 후 경과 시간 | seconds | 현재 시간 - 시나리오 시작 시간 |
| `scenario_step` | 현재 실행 중인 시나리오 단계 번호 | - | 시나리오별 로직에 따라 증가 |
| `dt` | 이전 TF 업데이트와의 시간 간격 | seconds | 현재 TF 시간 - 이전 TF 시간 |

### 제어 명령 필드

| 필드명 | 설명 | 범위 | 산출 방법 |
|--------|------|------|-----------|
| `accel_cmd` | 가속 페달 명령 | 0.0~1.0 | 시나리오 `update()` 메서드에서 계산 |
| `brake_cmd` | 브레이크 페달 명령 | 0.0~1.0 | 시나리오별 계산 (step_brake에서만 사용) |
| `steer_cmd` | 조향 명령 | -1.0~1.0 | 현재 모든 시나리오에서 0.0 |

### Ground Truth 필드 (시뮬레이터 제공)

| 필드명 | 설명 | 단위 | 데이터 소스 |
|--------|------|------|-------------|
| `true_velocity_x` | 실제 차량 속도 (x축) | m/s | `/Competition_topic` (EgoVehicleStatus.velocity.x) |
| `true_accel` | 실제 가속 페달 위치 | 0.0~1.0 | `/Competition_topic` (EgoVehicleStatus.accel) |
| `true_brake` | 실제 브레이크 페달 위치 | 0.0~1.0 | `/Competition_topic` (EgoVehicleStatus.brake) |
| `true_wheel_angle` | 실제 바퀴 각도 | radians | `/Competition_topic` (EgoVehicleStatus.wheel_angle) |

### IMU 센서 필드

| 필드명 | 설명 | 단위 | 데이터 소스 |
|--------|------|------|-------------|
| `imu_linear_accel_x` | IMU x축 선형 가속도 | m/s² | `/imu` (Imu.linear_acceleration.x) |
| `imu_linear_accel_y` | IMU y축 선형 가속도 | m/s² | `/imu` (Imu.linear_acceleration.y) |
| `imu_linear_accel_z` | IMU z축 선형 가속도 | m/s² | `/imu` (Imu.linear_acceleration.z) |
| `imu_angular_vel_x` | IMU x축 각속도 | rad/s | `/imu` (Imu.angular_velocity.x) |
| `imu_angular_vel_y` | IMU y축 각속도 | rad/s | `/imu` (Imu.angular_velocity.y) |
| `imu_angular_vel_z` | IMU z축 각속도 | rad/s | `/imu` (Imu.angular_velocity.z) |

### GPS 센서 필드

| 필드명 | 설명 | 단위 | 데이터 소스 |
|--------|------|------|-------------|
| `gps_latitude` | GPS 위도 | degrees | `/gps` (GPSMessage.latitude) |
| `gps_longitude` | GPS 경도 | degrees | `/gps` (GPSMessage.longitude) |
| `gps_altitude` | GPS 고도 | meters | `/gps` (GPSMessage.altitude) |

### TF 기반 위치 필드

| 필드명 | 설명 | 단위 | 산출 방법 |
|--------|------|------|-----------|
| `x_position` | 맵 좌표계 x 위치 | meters | TF lookup: `map` → `base_link` 변환의 translation.x |
| `y_position` | 맵 좌표계 y 위치 | meters | TF lookup: `map` → `base_link` 변환의 translation.y |
| `z_position` | 맵 좌표계 z 위치 | meters | TF lookup: `map` → `base_link` 변환의 translation.z |

### 속도 추정 필드

| 필드명 | 설명 | 단위 | 산출 방법 |
|--------|------|------|-----------|
| `estimated_velocity_x` | TF 기반 x축 속도 | m/s | TF 위치의 시간 미분 + 저역 필터 (α=0.7) |
| `estimated_velocity_y` | TF 기반 y축 속도 | m/s | TF 위치의 시간 미분 + 저역 필터 (α=0.7) |
| `estimated_velocity_z` | TF 기반 z축 속도 | m/s | TF 위치의 시간 미분 + 저역 필터 (α=0.7) |
| `imu_velocity_x` | IMU 적분 x축 속도 | m/s | IMU 가속도 적분 + 적응형 바이어스 보정 |
| `imu_velocity_y` | IMU 적분 y축 속도 | m/s | IMU 가속도 적분 + 적응형 바이어스 보정 |
| `imu_velocity_z` | IMU 적분 z축 속도 | m/s | IMU 가속도 적분 + 적응형 바이어스 보정 |
| `gps_velocity_x` | GPS 차분 x축 속도 | m/s | GPS 위치 차분 + 저역 필터 (α=0.3) |
| `gps_velocity_y` | GPS 차분 y축 속도 | m/s | GPS 위치 차분 + 저역 필터 (α=0.3) |
| `gps_velocity_z` | GPS 차분 z축 속도 | m/s | GPS 고도 차분 + 저역 필터 (α=0.3) |

### 유효성 플래그

| 필드명 | 설명 | 값 | 의미 |
|--------|------|-----|------|
| `is_valid` | 데이터 유효성 플래그 | 0/1 | TF 조회 성공 여부 (1: 성공, 0: 실패) |

## 속도 추정 알고리즘 상세

### 1. TF 기반 속도 추정 (estimated_velocity_*)
```python
# 위치 미분
velocity = (current_position - last_position) / dt

# 저역 필터 적용 (LowPassFilter 클래스 사용)
filter = LowPassFilter(alpha=0.7)
estimated_velocity = filter.update(velocity)
```

### 2. IMU 기반 속도 추정 (imu_velocity_*)
```python
# AdaptiveBiasEstimator 사용
bias_estimator = AdaptiveBiasEstimator()
corrected_accel = bias_estimator.update(accel, is_stationary)

# 가속도 적분
imu_velocity += corrected_accel * dt

# 정지 상태에서 드리프트 방지
if is_stationary:
    imu_velocity *= 0.95
```

### 3. GPS 기반 속도 추정 (gps_velocity_*)
```python
# GPS 좌표 변환 (conversions.py)
position_m = gps_to_meters(lat, lon, alt)

# 위치 차분
velocity = (position_m - last_position_m) / dt

# 저역 필터 적용 (GPS 노이즈 감소)
filter = LowPassFilter(alpha=0.3)
gps_velocity = filter.update(velocity)
```

## 모니터링

실시간 데이터 모니터링:
```bash
# 시스템 데이터 확인
rostopic echo /control_system_data

# PlotJuggler를 사용한 시각화
rosrun plotjuggler plotjuggler
# /control_system_data 토픽 구독

# rqt_plot을 사용한 실시간 그래프
rqt_plot /control_system_data/true_velocity_x /control_system_data/accel_cmd
```

## 데이터 분석

수집된 CSV 파일은 Python, MATLAB, 또는 다른 데이터 분석 도구에서 직접 사용할 수 있습니다.

### Python 예제
```python
import pandas as pd
import matplotlib.pyplot as plt

# 데이터 로드
df = pd.read_csv('data/step_accel_20250806_153946.csv')

# 시간-속도 그래프
plt.figure(figsize=(12, 6))
plt.plot(df['scenario_time'], df['true_velocity_x'], label='True Velocity')
plt.plot(df['scenario_time'], df['estimated_velocity_x'], label='TF Estimated')
plt.plot(df['scenario_time'], df['imu_velocity_x'], label='IMU Estimated')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)
plt.show()

# 입력-출력 관계
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(df['scenario_time'], df['accel_cmd'], 'b-', label='Accel Command')
plt.ylabel('Command')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(df['scenario_time'], df['true_velocity_x'], 'r-', label='Velocity Response')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
```

## 테스트

### 단위 테스트 예제
```python
# 시나리오 로직을 ROS 없이 테스트
from data_collection.scenario.step_accel import StepAccelScenario

# 시나리오 생성
scenario = StepAccelScenario()
scenario.setup({'step_duration': 5.0, 'rest_duration': 2.0})

# 시작
scenario.start(0.0)

# 2.5초 후 제어 명령 확인
accel, brake, steer = scenario.update(2.5)
assert accel == 0.2  # 첫 번째 스텝
assert brake == 0.0
assert steer == 0.0

# 완료 확인
assert not scenario.is_complete()
```

## 성능 고려사항

- **버퍼링된 로깅**: I/O 오버헤드 감소 (1000개 단위 쓰기)
- **필터 캐싱**: 중복 계산 방지
- **모듈형 로딩**: 선택적 import로 메모리 효율성
- **스레드 안전성**: 매뉴얼 제어와 메인 루프 분리

## 주의사항

1. **초기화**: 데이터 수집 시작 전 차량이 완전히 정지 상태인지 확인
2. **센서 동기화**: IMU와 GPS 데이터의 타임스탬프가 올바른지 확인
3. **TF 유효성**: `is_valid` 필드를 확인하여 TF 데이터가 유효한 구간만 사용
4. **단위 변환**: 속도 데이터는 m/s 단위, 필요시 km/h로 변환 (×3.6)
5. **브레이크 테스트**: 초기 속도 빌드업 시 차량 주변 안전 확인

## 문제 해결

### TF lookup 실패
- `/tf` 토픽이 발행되고 있는지 확인
- `map` 프레임과 `base_link` 프레임이 존재하는지 확인
```bash
rosrun tf view_frames
```

### IMU 드리프트
- 차량 정지 상태에서 바이어스 보정이 제대로 작동하는지 확인
- `AdaptiveBiasEstimator`의 파라미터 조정 고려

### GPS 노이즈
- GPS 신호 품질 확인
- 필터 파라미터 (α) 조정: `utils/filters.py`의 LowPassFilter

### 시나리오가 시작되지 않음
- 차량 초기화 상태 확인
- 인터랙티브 모드에서 'i' 키로 상태 확인

## 향후 개선 계획

1. **Dynamic Reconfigure**: 런타임 파라미터 조정 (`rqt_reconfigure`)
2. **Service Interface**: CLI를 ROS 서비스/액션으로 대체
3. **Plugin System**: 설정 파일에서 시나리오 동적 로드
4. **Multi-format Logging**: rosbag, HDF5, Parquet 지원
5. **Real-time Monitoring**: Grafana/Prometheus 통합
6. **시나리오 체이닝**: 여러 시나리오 순차 실행
7. **자동 분석**: 데이터 수집 후 자동 시스템 식별

## 기여 및 문의

이 패키지는 자율주행 차량 MPC 시스템 개발 프로젝트의 일부입니다.
문제나 제안사항이 있으면 이슈를 등록해 주세요.

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.