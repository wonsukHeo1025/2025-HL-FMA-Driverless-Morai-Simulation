## Topics

### `/ctrl_cmd` 토픽

`morai_msgs/CtrlCmd`

- Ego 차량을 제어하기 위해 발행하는 메시지

```bash
❯ rosmsg show morai_msgs/CtrlCmd
int32 longlCmdType   # 제어 방식을 결정하는 인덱스
	    # longlCmdType == 1: Throttle제어. accel/brake/steering만 사용
	    # longlCmdType == 2: Velocity제어. velocity/steering만 사용
	    # longlCmdType == 3: Acceleration제어. acceleration/steering만 사용
float64 accel        # 차량의 엑셀값을 의미하며 0 ~ 1 범위를 가짐
float64 brake        # 차량의 브레이크값을 의미하며 0 ~ 1 범위를 가짐
float64 steering     # 차량의 바퀴 각도 (rad)
float64 velocity     # longCmdType이 2일 경우 사용 (km/h)
float64 acceleration # longCmdType이 3일 경우 사용 (m/s^2)
```

### `/Object_topic` 토픽

`morai_msgs/ObjectStatusList`

- 사용자가 배치한 주변 물체 정보를 나타내는 메시지

```bash
❯ rosmsg show morai_msgs/ObjectStatusList
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
int32 num_of_npcs                  # 사용자가 배치한 NPC 차량의 개수
int32 num_of_pedestrian            # 사용자가 배치한 보행자의 개수
int32 num_of_obstacle              # 사용자가 배치한 장애물의 개수
morai_msgs/ObjectStatus[] npc_list # NPC 차량 정보 리스트
  int32 unique_id                  # 물체의 unique id 값
  int32 type                       # 물체의 타입
  string name                      # 물체의 이름
  float64 heading                  # 물체의 헤딩(deg)
  geometry_msgs/Vector3 velocity   # 물체의 속도 벡터
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 acceleration # 물체의 가속도 벡터 (차량 기준 좌표계)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 size     # 물체의 크기 벡터 (m)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 position   # 물체의 위치 벡터
    float64 x
    float64 y
    float64 z
morai_msgs/ObjectStatus[] pedestrian_list # 보행자 정보 리스트
  int32 unique_id                  # 물체의 unique id 값
  int32 type                       # 물체의 타입
  string name                      # 물체의 이름
  float64 heading                  # 물체의 헤딩(deg)
  geometry_msgs/Vector3 velocity   # 물체의 속도 벡터
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 acceleration # 물체의 가속도 벡터 (차량 기준 좌표계)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 size     # 물체의 크기 벡터 (m)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 position   # 물체의 위치 벡터
    float64 x
    float64 y
    float64 z
morai_msgs/ObjectStatus[] obstacle_list # 장애물 정보 리스트
  int32 unique_id                  # 물체의 unique id 값
  int32 type                       # 물체의 타입
  string name                      # 물체의 이름
  float64 heading                  # 물체의 헤딩(deg)
  geometry_msgs/Vector3 velocity   # 물체의 속도 벡터
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 acceleration # 물체의 가속도 벡터 (차량 기준 좌표계)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 size     # 물체의 크기 벡터 (m)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 position   # 물체의 위치 벡터
    float64 x
    float64 y
    float64 z
```

### `/CollisionData` 토픽

`morai_msgs/CollisionData`

- Ego 차량과의 충돌 데이터를 나타내는 메시지

```bash
❯ rosmsg show morai_msgs/CollisionData
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float64 global_offset_x
float64 global_offset_y
float64 global_offset_z
morai_msgs/ObjectStatus[] collision_object # 충돌된 오브젝트 정보
  int32 unique_id                          # 물체의 unique id 값
  int32 type                               # 물체의 타입
                                           # 0: Pedestrian
                                           # 1: Surround Vehicle (NPC)
                                           # 2: Obstacle
  string name                              # 물체의 이름
  float64 heading                          # 물체의 헤딩(deg)
  geometry_msgs/Vector3 velocity           # 물체의 속도 벡터
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 acceleration       # 물체의 가속도 벡터 (차량 기준 좌표계)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 size             # 물체의 크기 벡터 (m)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 position           # 물체의 위치 벡터
    float64 x
    float64 y
    float64 z
```

### `/Competition_topic` 토픽

`morai_msgs/EgoVehicleStatus` 

- `velocity.x`, `accel`, `brake`, `wheel_angle` 필드만 사용 가능
- `velocity.x` 필드 값은 m/s
    - 100kmph = 27.7777mps
- `wheel_angle` 필드 값은 -40.0 - 40.0 까지 degree

```jsx
❯ rosmsg show morai_msgs/EgoVehicleStatus
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
int32 unique_id
geometry_msgs/Vector3 acceleration
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 position
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 velocity
  float64 x
  float64 y
  float64 z
float64 heading
float32 accel
float32 brake
float32 wheel_angle
float32 lateral_offset
float32 tire_lateral_force_fl
float32 tire_lateral_force_fr
float32 tire_lateral_force_rl
float32 tire_lateral_force_rr
float32 side_slip_angle_fl
float32 side_slip_angle_fr
float32 side_slip_angle_rl
float32 side_slip_angle_rr
float32 tire_cornering_stiffness_fl
float32 tire_cornering_stiffness_fr
float32 tire_cornering_stiffness_rl
float32 tire_cornering_stiffness_rr
```

### 기타 토픽 인터페이스

`Morai_msgs/GPSMessage` 

```bash
❯ rosmsg show morai_msgs/GPSMessage
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float64 latitude
float64 longitude
float64 altitude
float64 eastOffset
float64 northOffset
int16 status
```

이외 IMU, 오도메트리, 이미지, 포인트클라우드는 ROS 공식 인터페이스 사용

## Services

### `/Service_MoraiEventCmd`

- 기어, 제어모드 등의 이벤트 제어 요청 메시지

```bash
❯ rossrv show morai_msgs/MoraiEventCmdSrv
morai_msgs/EventInfo request
  int8 option            # 이벤트 제어를 요청하는 필드 옵션
                         # 1 : ctrl_mode
                         # 2 : gear
  int32 ctrl_mode        # 차량의 control mode 제어
                         # 1 : Keyboard
                         # 3 : ExternalCtrl
  int32 gear             # 차량의 기어 변경
                         # 1 : P
                         # 2 : R
                         # 3 : N
                         # 4 : D
  morai_msgs/Lamps lamps
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    int8 turnSignal
    int8 emergencySignal
  bool set_pause
---
morai_msgs/EventInfo response
  int8 option
  int32 ctrl_mode
  int32 gear
  morai_msgs/Lamps lamps
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    int8 turnSignal
    int8 emergencySignal
  bool set_pause
```