# **Phase 3: ROS 노드 구현 및 통합 (v2)**

## **1. 개요**

본 문서는 `Phase 2`에서 설계된 `MpcCore` 제어기 로직을 실제 MORAI 시뮬레이터 환경과 연동하기 위한 ROS 노드의 구현 계획을 상세히 정의합니다.

이 노드는 **상태 입력 소스를 선택**할 수 있는 기능을 포함하여, `Phase 4`에서 진행될 3단계 검증 워크플로우를 체계적으로 지원하도록 설계됩니다.

## **2. 노드 아키텍처**

-   **노드 이름**: `mpc_controller_node.py`
-   **핵심 기능**: ROS 파라미터(`~state_source`) 값에 따라 현재 상태(속도)를 가져오는 소스를 동적으로 변경합니다.

```text
                                          +----------------------------+
                               +--------> |   TF Listener & Processor  | --+
                               |          +----------------------------+   |
                               |                                           v
[ROS Param: ~state_source] ----+          +----------------------------+   +---> [MpcCore] -> [Publisher] -> [/ctrl_cmd]
                               |          |      State Estimator       |   |
                               +--------> | (e.g., robot_localization) | --+
                                          |  Subscribes to /odom       |
                                          +----------------------------+
```

-   **`state_source == "tf"` (1단계: 제어기 코어 검증용)**:
    -   노드는 `tf2_ros` 리스너를 사용하여 `map` -> `base_link` 변환 정보를 실시간으로 조회합니다.
    -   조회된 위치 정보를 시간으로 미분하여 **이상적이고 완벽한 속도**를 계산하고, 이를 `MpcCore`에 입력합니다.
-   **`state_source == "estimator"` (3단계: 통합 시스템 검증용)**:
    -   노드는 외부 상태 추정기(e.g., `robot_localization`)가 발행하는 `/odometry/filtered` 토픽을 구독합니다.
    -   해당 메시지의 `twist.twist.linear.x` 필드에서 **현실적인 추정 속도**를 가져와 `MpcCore`에 입력합니다.

## **3. 노드 상세 설계**

### **3.1. ROS 파라미터**

-   **`~state_source`** (string, default: "tf"):
    -   제어기가 사용할 상태 정보의 소스를 지정합니다.
    -   "tf": TF 미분을 통해 이상적인 상태를 사용합니다.
    -   "estimator": 외부 상태 추정기의 토픽을 구독하여 현실적인 상태를 사용합니다.
-   **`~control_rate`** (double, default: 50.0): 제어 루프의 실행 주기를 Hz 단위로 지정합니다.

### **3.2. 구독 (Subscriptions)**

-   **공통 구독**:
    -   `/target_velocity` (`std_msgs/Float64`): 외부로부터 목표 속도를 수신합니다.
-   **`state_source == "tf"` 일 때**:
    -   내부적으로 `tf2_ros.Buffer`와 `tf2_ros.TransformListener`를 사용합니다. `/tf`, `/tf_static` 토픽을 구독합니다.
-   **`state_source == "estimator"` 일 때**:
    -   `/odometry/filtered` (`nav_msgs/Odometry`): 상태 추정기가 발행하는 위치/속도/자세 정보를 구독합니다.

### **3.3. 발행 (Publications)**

-   **/ctrl_cmd** (`morai_msgs/CtrlCmd`): MPC 제어기가 계산한 최종 제어 입력을 차량 시뮬레이터에 전달합니다.

### **3.4. 제어 루프 로직**

ROS `Timer`를 사용하여 `control_rate`에 설정된 주기로 메인 제어 루프를 실행합니다.

```python
# mpc_controller_node.py

class MpcControllerNode:
    def __init__(self):
        # ... (ROS 초기화, Pub/Sub, 파라미터 로드) ...
        self.state_source = rospy.get_param("~state_source", "tf")

        # MpcCore 인스턴스 생성
        self.mpc = MpcCore(...)

        # 상태 소스에 따른 초기화
        if self.state_source == "tf":
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.last_tf_time = None
            self.last_tf_pos = None
        elif self.state_source == "estimator":
            self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        # ...

    def get_velocity_from_tf(self):
        # tf_buffer에서 'map' -> 'base_link' transform 조회
        # 현재 위치와 이전 위치, 시간 차이를 이용해 속도 계산
        # ...
        return calculated_velocity

    def odom_callback(self, msg):
        # Odometry 메시지에서 속도 정보 업데이트
        self.current_velocity = msg.twist.twist.linear.x

    def control_loop(self, event):
        # 1. 현재 속도 가져오기
        if self.state_source == "tf":
            self.current_velocity = self.get_velocity_from_tf()
        # 'estimator' 모드에서는 콜백에서 self.current_velocity가 이미 업데이트됨

        # 2. 목표 궤적 생성
        target_trajectory = ...

        # 3. MPC 문제 풀이
        u_star = self.mpc.solve(self.current_velocity, target_trajectory)

        # 4. 제어 메시지 발행
        self.ctrl_cmd_pub.publish(...)

# ... (main 함수) ...
```

## **5. 결론 및 다음 단계**

본 설계안에 따라 `mpc_controller_node.py`를 구현하면, 간단한 파라미터 변경만으로 제어기의 입력 소스를 전환할 수 있습니다. 이는 `Phase 4`의 체계적인 3단계 검증 워크플로우를 수행하는 데 필수적인 기능입니다.

**다음 단계**:
-   `Phase 4: 검증 및 튜닝` 계획에 따라, 이 노드를 사용하여 각 단계별 검증을 순차적으로 수행합니다.
