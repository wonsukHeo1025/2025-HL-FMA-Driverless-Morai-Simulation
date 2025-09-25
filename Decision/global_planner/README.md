# Global Planner

## Overview

Global Planner는 MORAI 시뮬레이터 환경에서 자율주행 차량의 전역 경로 계획 및 관리를 담당하는 통합 시스템입니다. GPS 기반 실시간 위치 추적과 맵 좌표계 기반 경로 생성을 결합하여 효율적이고 확장 가능한 아키텍처를 제공합니다.

## Key Features

### 🚀 Core Features
- **GPS 기반 실시간 위치 추적**: MORAI GPS 토픽 활용
- **맵 좌표계 우선 접근**: 시뮬레이터 맵 좌표 직접 사용
- **통합 경로 관리**: 글로벌/로컬 경로 생성 및 속도 계획
- **모듈화 설계**: 최소한의 오버엔지니어링

### 🔧 Advanced Features
- **IMU/Dead Reckoning 통합**: 센서 융합 지원 (확장 가능)
- **실시간 경로 최적화**: 곡률 기반 속도 프로파일링
- **다중 포맷 지원**: txt/csv 경로 파일 자동 변환
- **ROS1 표준 인터페이스**: 기존 시스템과의 호환성

## Architecture

```
Global Planner Package
├── Core Components
│   ├── Unified Path Planner (Python)
│   ├── Path Converter (Python)
│   └── Path Utils (Python)
├── Integration Modules
│   ├── IMU Processor (Python)
│   ├── Dead Reckoning (Python)
│   └── Sensor Fusion (Python)
├── ROS Interfaces
│   ├── GPS Subscriber
│   ├── Path Publishers
│   └── TF Broadcasters
└── Configuration
    ├── YAML Config Files
    ├── Launch Files
    └── Documentation
```

## Quick Start

### 1. Prerequisites
```bash
# Required ROS packages
sudo apt-get install ros-noetic-nav-msgs ros-noetic-geometry-msgs ros-noetic-visualization-msgs

# Python dependencies
pip install numpy scipy pandas
```

### 2. Build Package
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Convert Path File (Optional)
```bash
# txt 파일을 CSV로 변환
rosrun global_planner path_converter.py --input data/25hl_global_path_ver3.txt --output data/25hl_global_path_ver3.csv
```

### 4. Launch Global Planner
```bash
# 기본 모드
roslaunch global_planner global_planner.launch

# IMU 통합 모드 (확장)
roslaunch global_planner global_planner_with_imu.launch
```

## ROS Interface

### Subscribers
- `/gps` (morai_msgs/GPSMessage): GPS 위치 정보
- `/Ego_topic` (morai_msgs/EgoVehicleStatus): 차량 상태
- `/imu/data` (sensor_msgs/Imu): IMU 데이터 (선택적)

### Publishers
- `/planning/global/path` (nav_msgs/Path, latched): 전역 경로
- `/planning/global/path_default|second|third` (nav_msgs/Path, latched): 프리뷰 경로
- `/planning/global/recommended_path` (std_msgs/String): 추천 경로 이름(default|second|third)
- `/planning/speed_profile/global` (std_msgs/Float32MultiArray, latched): 전역 속도 프로파일
- `/vis/planning/global/path_corridors` (MarkerArray): Corridor 시각화

### Parameters
```yaml
# Path Configuration
path_file: "data/25hl_global_path_ver3.csv"  # 경로 파일
path_topic: "/planning/global/path"          # 전역 경로 토픽 (canonical)
local_path_topic: "/local_path"              # 로컬 경로 토픽

# Processing Configuration
publish_rate: 10.0                          # 발행 주기 (Hz)
lookahead_distance: 50.0                    # 전방 주시 거리 (m)
resample_spacing: 0.2                       # 경로 리샘플링 간격 (m)
smoothing_window: 11                        # 평활화 윈도우 크기

# Speed Configuration
max_speed: 20.0                             # 최대 속도 (m/s)
min_speed: 5.0                              # 최소 속도 (m/s)

# Frame Configuration
map_frame: "map"                            # 맵 프레임
base_frame: "base_link"                     # 차량 베이스 프레임
gps_frame: "gps"                            # GPS 프레임

# Sensor Integration
use_gps: true                               # GPS 사용
use_imu: false                              # IMU 사용
use_dead_reckoning: false                   # Dead Reckoning 사용

# Debug Configuration
publish_markers: false                      # 마커 발행(운영 기본값은 비활성)
enable_logging: true                        # 로깅 활성화
```

## Usage Examples

### Basic Path Planning
```python
from global_planner.scripts.unified_path_planner import UnifiedPathPlanner

# Global Planner 초기화
planner = UnifiedPathPlanner()

# 경로 로드 및 처리
planner.load_and_process_path()

# 실시간 경로 발행 시작
planner.run()
```

### Path Conversion
```python
from global_planner.scripts.path_converter import PathConverter

# 변환기 초기화
converter = PathConverter()

# txt → CSV 변환
df = converter.txt_to_dataframe('path.txt')
converter.dataframe_to_csv(df, 'path.csv')

# 통계 출력
converter.print_statistics()
```

### Custom Configuration
```yaml
# config/global_planner_custom.yaml
path_file: "data/custom_path.csv"
max_speed: 15.0
lookahead_distance: 30.0
use_imu: true
```

## Coordinate Systems

### GPS Coordinate System
MORAI 시뮬레이터의 GPS 메시지 구조:
```cpp
float64 latitude    // 위도
float64 longitude   // 경도
float64 altitude    // 고도
float64 eastOffset  // 동쪽 오프셋 (맵 기준)
float64 northOffset // 북쪽 오프셋 (맵 기준)
```

### Map Coordinate System
시뮬레이터 맵 좌표계:
```cpp
// GPS → 맵 좌표 변환
double map_x = longitude;  // 실제로는 eastOffset 사용
double map_y = latitude;   // 실제로는 northOffset 사용
```

## Performance Optimization

### Target Performance
- **Path Loading**: < 100ms
- **Local Path Generation**: < 10ms
- **Publishing Rate**: 10Hz 이상
- **Memory Usage**: < 50MB
- **CPU Usage**: < 5% (단일 코어)

### Optimization Techniques
- **NumPy 기반 계산**: 벡터화 연산으로 성능 최적화
- **Selective Publishing**: 필요한 토픽만 선택적 발행
- **Threading Safety**: ROS 콜백과 메인 루프 간 동기화
- **Memory Pool**: 자주 사용하는 객체 재사용

## Testing and Validation

### Unit Tests
```bash
# Python 유닛 테스트 실행
python -m pytest test/test_unified_path_planner.py

# ROS 통합 테스트
rostest global_planner test_global_planner.test
```

### Performance Monitoring
```bash
# 성능 통계 모니터링
rostopic hz /planning/global/path

# RViz에서 경로 시각화
rosrun rviz rviz -d $(rospack find global_planner)/rviz/global_planner.rviz
```

## Troubleshooting

### Common Issues

#### 1. Path File Not Found
```bash
# 경로 파일 존재 확인
ls $(rospack find global_planner)/data/

# 권한 설정
chmod +x $(rospack find global_planner)/scripts/*.py
```

#### 2. GPS Data Not Received
```bash
# GPS 토픽 확인
rostopic list | grep gps
rostopic echo /gps

# GPS 노드 실행 확인
rosnode list | grep gps
```

#### 3. Performance Issues
```bash
# CPU/메모리 사용량 확인
top -p $(pgrep -f unified_path_planner)

# ROS 토픽 주기 확인
rostopic hz /planning/global/path
```

### Debug Mode
```bash
# 디버그 로깅 활성화
roslaunch global_planner global_planner.launch enable_logging:=true

# 마커 발행 활성화 (전역 속도 프로파일 시각화)
roslaunch global_planner global_planner_with_imu.launch publish_markers:=true
```

## Future Extensions

### 🔮 Planned Features
- **A* 경로 계획**: 동적 장애물 회피
- **머신러닝 통합**: 경로 예측 및 최적화
- **다중 목표 지점**: 복잡한 미션 지원
- **SLAM 통합**: 실시간 맵 업데이트

### 📊 Advanced Sensor Fusion
- **카메라 통합**: 비전 기반 위치 보정
- **LiDAR 통합**: 3D 장애물 감지
- **GNSS/IMU 복합 내비게이션**: 고정밀 위치 추정

## Contributing

### Development Setup
```bash
# 가상환경 설정 (선택적)
python3 -m venv global_planner_env
source global_planner_env/bin/activate

# 개발 의존성 설치
pip install -r requirements-dev.txt

# 코드 포맷팅
black scripts/*.py
isort scripts/*.py
```

### Code Standards
- **Python**: PEP 8 준수, 타입 힌트 사용
- **ROS**: 표준 메시지 타입 사용
- **Documentation**: docstring 및 주석 완성
- **Testing**: 유닛 테스트 80% 이상 커버리지

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- MORAI 시뮬레이터 팀
- ROS 커뮤니티
- 자율주행 연구자들

## Contact

For questions and support:
- **Email**: global_planner@team.com
- **Issues**: GitHub Issues
- **Documentation**: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
