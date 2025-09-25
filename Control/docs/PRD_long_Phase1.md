# **Phase 1: 시스템 모델링을 위한 데이터 수집 (v5)**

## **1. 개요 및 워크플로우에서의 위치**

본 문서는 MPC(Model Predictive Control) 제어기 개발에 필요한 **차량 동역학 모델의 파라미터($A, B, d$)를 추정**하기 위한 데이터 수집 계획을 정의합니다.

이 단계는 **`Phase 4`에서 진행될 3단계 검증 및 튜닝 워크플로우의 가장 중요한 선행 작업**입니다. 여기서 수집된 데이터의 품질이 전체 제어 시스템의 성능을 좌우하므로, 현실적인 센서 데이터를 기반으로 정확한 시스템 모델을 구축하는 것을 목표로 합니다.

**핵심 원칙**:
1.  **목적**: 제어 입력(`accel`/`brake`)과 그로 인한 차량의 실제 물리적 반응(IMU/GPS 센서 값의 변화) 사이의 관계를 나타내는 모델을 만듭니다.
2.  **신뢰 소스**: **오직 `IMU`와 `GPS` 원시 데이터만을 신뢰**하며, 이를 바탕으로 모델링에 필요한 상태(속도 등)를 추정합니다.
3.  **ROS 컨벤션 준수**: 다중 센서는 각각 고유한 토픽 이름을 가집니다.
4.  **통합 모니터링**: `PlotJuggler`를 통한 통합 디버깅이 용이하도록 종합 모니터링 메시지를 설계하고 활용합니다.
5.  **발행 주기**: 모든 제어 및 데이터 발행 주기는 **50Hz**로 설정합니다.

## **2. 시스템 아키텍처 및 데이터 흐름**

```text
                               +-----------------------------+
[IMU/GPS 다중 토픽] ---------> |                             | ---------> [/ctrl_cmd]
                               |   data_collection_node.py   |
[data_collection_core.py] ---> |        (ROS 환경)           | ---------> [/control_system_data]
  (시나리오 로직)              |                             |
                               +---------------+-------------+
                                               |
                                               v
                                     [data_logger.py] ---------> [log.csv]
                                       (CSV 저장 로직)
```

-   **`data_collection_node.py`의 역할**:
    -   50Hz 주기로 제어 루프를 실행합니다.
    -   `data_collection_core`로부터 시나리오에 따른 `accel`, `brake`, `steer` 명령 값을 받아 `/ctrl_cmd` 토픽으로 차량에 발행합니다.
    -   **다중 센서 토픽 구독**: `/sensor/imu/rear_axle`, `/sensor/imu/front_bumper`, `/sensor/gps/rear_axle` 등 모든 개별 센서 토픽을 각각 구독합니다.
    -   **`message_filters.TimeSynchronizer`**를 사용하여 모든 센서 데이터의 타임스탬프를 정밀하게 동기화하고, 하나의 콜백 함수에서 처리합니다.
    -   동기화된 원시 데이터를 기반으로 **속도를 직접 추정**합니다.
    -   동기화된 콜백이 실행될 때마다, 생성된 명령, 모든 원시 센서 데이터, 추정된 상태 값을 하나의 종합 메시지에 담아 `/control_system_data` 토픽으로 발행합니다.
    -   오프라인 분석을 위해 모든 데이터를 `log.csv` 파일에 상세히 기록합니다.

## **3. 통합 모니터링을 위한 Custom Message**

디버깅 편의성을 극대화하기 위해, 관련된 모든 데이터를 하나의 메시지에 통합합니다.

-   **신규 패키지 이름**: `control_msgs`
-   **패키지 위치**: `Control/control_msgs/`
-   **메시지 파일 위치**: `Control/control_msgs/msg/SystemIdData.msg`
-   **메시지 정의 (`SystemIdData.msg`)**:
    ```
    # Comprehensive data bus for control system debugging and monitoring (v4)

    Header header

    # --- 1. Control Commands (from Scenario Node) ---
    float64 accel_cmd
    float64 brake_cmd
    float64 steer_cmd

    # --- 2. Raw Sensor Data (from Sensor Topics) ---
    # The subscriber node will fill these fields from their respective topics.
    sensor_msgs/Imu imu_rear_axle
    sensor_msgs/Imu imu_front_bumper
    sensor_msgs/Imu imu_left_mirror
    sensor_msgs/Imu imu_right_mirror

    morai_msgs/GPSMessage gps_rear_axle
    morai_msgs/GPSMessage gps_front_bumper

    # --- 3. Estimated States (Calculated in this or other nodes) ---
    float64 estimated_velocity_imu_rear  # Velocity from rear axle IMU
    float64 estimated_velocity_gps_rear  # Velocity from rear axle GPS

    # --- 4. Contextual Information ---
    string scenario_type
    ```
-   **토픽 이름**: `/control_system_data`
-   **발행 주기**: 50Hz

## **4. 데이터 수집 시나리오**

### **4.1. 센서 토픽 및 `frame_id` 명명 규칙**

-   **토픽 이름**: `/<sensor_type>/<location>` 형태의 계층적 이름을 사용합니다.
    -   `/sensor/imu/rear_axle`, `/sensor/imu/front_bumper`, ...
    -   `/sensor/gps/rear_axle`, `/sensor/gps/front_bumper`
-   **`frame_id`**: TF 트리와 연동되는 링크 이름을 사용합니다.
    -   `imu_rear_axle_link`, `imu_front_bumper_link`, ...
    -   `gps_rear_axle_link`, `gps_front_bumper_link`

### **4.2. 자동화된 실험 절차**

하나의 자동화된 시퀀스로 일관성을 확보하며, 총 실험 시간은 약 5분입니다.

1.  **초기화 및 카운트다운 (0 ~ 10초)**
    -   터미널에 "데이터 수집을 5초 후에 시작합니다..." 카운트다운 출력.
    -   차량을 정지 상태(`accel=0, brake=1`)로 유지하여 초기 상태 안정화.
2.  **다단계 가속 스텝 응답 (10 ~ 90초)**
    -   `accel` 값을 0.2부터 1.0까지 0.2씩 증가. 각 스텝은 10초간 유지하고, 스텝 사이에 5초간 휴지(`accel=0, brake=0`).
3.  **다단계 감속 스텝 응답 (90 ~ 170초)**
    -   `accel=0.8`로 10초간 가속하여 고속 상태 도달. 이후 `brake` 값을 0.2부터 1.0까지 0.2씩 증가. 각 스텝은 8초간 유지하고, 스텝 사이에 다시 고속 상태로 복귀.
4.  **PRBS (Pseudo-Random Binary Sequence) (170 ~ 230초)**
    -   `accel` 입력을 두 레벨(예: `0.3`과 `0.7`) 사이에서 무작위한 시간 간격(0.5초 ~ 2.0초)으로 60초간 전환.
5.  **처프 신호 (Chirp Signal) (230 ~ 290초)**
    -   `accel` 입력을 사인파 형태로 인가하되, 주파수를 60초에 걸쳐 점진적으로 증가시킴 (0.1Hz → 2Hz).
    -   입력 신호: $u(t) = 0.4 + 0.3 \cdot \sin(2\pi f(t) \cdot t)$
6.  **종료 및 데이터 저장 (290초 ~)**
    -   차량을 안전하게 정지(`accel=0, brake=1`).
    -   CSV 파일을 최종 저장하고 노드를 종료.

## **5. 구현 계획**

1.  **`Control/control_msgs` 패키지 생성**: `catkin_create_pkg control_msgs std_msgs sensor_msgs morai_msgs roscpp rospy`.
2.  **`Control/control_msgs/msg/SystemIdData.msg` 파일 생성**: 위 `v4` 메시지 정의 내용을 추가.
3.  **`Control/control_msgs/CMakeLists.txt` 및 `package.xml` 수정**:
    -   `find_package`에 `sensor_msgs`, `morai_msgs` 추가.
    -   `message_generation` 및 `message_runtime` 의존성 추가.
    -   `add_message_files`에 `SystemIdData.msg` 추가 및 `generate_messages()` 활성화.
4.  **전체 프로젝트 `catkin_make` 실행**: `control_msgs`가 빌드되고 메시지 파이썬 모듈이 생성되도록 함.
5.  **`data_collection_node.py` 구현**:
    -   `from control_msgs.msg import SystemIdData` 임포트.
    -   `message_filters`를 임포트하고, 모든 개별 센서 토픽에 대한 `Subscriber` 객체를 생성.
    -   `TimeSynchronizer`를 설정하여 모든 센서 메시지를 받는 단일 콜백 함수(`sync_callback`)를 지정.
    -   `sync_callback(self, imu_rear_msg, imu_front_msg, ...)` 함수 내에서:
        -   `SystemIdData` 메시지 객체 생성.
        -   가장 최근의 제어 명령(`accel_cmd`, ...)을 메시지에 채움.
        -   콜백으로 들어온 모든 센서 메시지를 통째로 메시지에 채움 (`imu_rear_axle = imu_rear_msg`, ...).
        -   IMU/GPS 데이터로 속도를 추정하고 그 결과를 메시지에 채움.
        -   완성된 `SystemIdData` 메시지를 `/control_system_data` 토픽으로 발행.
        -   CSV 로깅 수행.
    -   별도의 50Hz `Timer` 콜백에서는 시나리오에 따라 제어 명령을 계산하고 `/ctrl_cmd`를 발행.
