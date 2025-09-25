# 통합 MPC 컨트롤러 분석 및 개발 계획

## 1. 배경 및 목표
- 기존 ROS 패키지 `mpc_lateral_controller`(횡방향)와 `mpc_longitudinal_controller`(종방향)는 Python 기반으로 각각 독립 실행되며 토픽(`/lateral_mpc/control_info`)을 통해 상호 정보를 교환함.
- Python + CVXPY/OSQP 조합으로 이미 원하는 동작이 구현되어 있으나, 계산 지연을 줄이기 위해 C++ + osqpEigen 기반으로 재구현하고 두 기능을 하나의 패키지 `mpc_controller` 안에서 통합하려는 것이 목표.
- 통합 노드는 동일한 입력을 받아 내부에서 횡/종방향 MPC를 순차적으로 실행하고 `/ctrl_cmd` 출력과 시각화·디버그 토픽만 유지하도록 설계함.

## 2. 기존 패키지 분석 요약

### 2.1 `mpc_lateral_controller`
- **주요 노드**: `scripts/lateral_mpc_controller_node.py`
  - 초기화 시 파라미터 로드, 모델 선택(kinematic/dynamic), OSQP 백엔드 설정.
  - `PathProcessor`, `StateEstimator`를 활용하여 경로 전처리, 최근접점 계산, 상태 추정 수행.
  - 동적 예측 지평 조정: 곡률 ROI를 기반으로 Np를 가변적으로 변경하는 `_maybe_update_horizon` 로직 포함.
  - 시각화: 예측 경로(Path), 속도/조향 Pillar MarkerArray, 현재 인덱스 Marker 등을 `/vis/control/lateral_mpc/...` 네임스페이스에 발행.
  - 디버그/상태 토픽: `/lateral_mpc/control_info`(Custom ControlInfo), `/lateral_mpc/debug`, `/lateral_mpc/status`, `/lateral_mpc/status/verbose`(LateralMpcTiming), `/lateral_mpc/metrics`.
  - 종방향 예측 속도(Optional)를 Float32MultiArray로 입력받아 횡방향 예측 결과 시각화에 활용.
- **MPC 코어 구조** (`src/mpc_lateral_controller/*`)
  - `kinematic_mpc_core.py`: 2차 상태(ey, epsi) 기반 단순 모델, CVXPY/OSQP 하이브리드 구조, 밴드 소프트 제약, IRLS 가중치, 시간가변 Q/P 반영.
  - `lateral_mpc_core.py`: 4차 상태(ey, ey_dot, epsi, epsi_dot) 기반 동적 모델, 위와 유사한 가중치/제약 지원.
  - `osqp_backend_*.py`: 입력 변화율 제약, 밴드 슬랙 변수, 터미널 앵커링, state-dependent smoothing 등을 직접 QP로 구성하여 OSQP 호출.
  - `path_processor.py`: Path → 내장 리스트 변환, 세 점 곡률 계산, 최근접점 탐색.
  - `state_estimator.py`: TF / EgoVehicleStatus / IMU 등 다양한 정보 통합, 속도·가속 추정, 저속 홀드 관리.

### 2.2 `mpc_longitudinal_controller`
- **주요 노드**: `scripts/mpc_node.py`
  - 속도 프로파일(Float32MultiArray)을 수신해 0.5m → 0.05m 선형 보간 후 내부 버퍼 저장.
  - Lateral `ControlInfo`를 구독하여 현재 경로 인덱스 및 스티어링 값을 활용, 내부 통합 모드에서 `/ctrl_cmd.steering`에 반영.
  - 저속 모드 스위칭: 속도 조건 기반으로 Nc, Q, R, R_delta, 입력 한계 등을 런타임 변경.
  - Zero-throttle, stop-window, stopline hold 등 거리 및 속도 기반 보정 로직 존재.
  - 디버그 토픽: 현재 속도/목표 속도 Float64, solver status String, 예측 속도 시퀀스(Float32MultiArray), LateralMpcTiming 재활용.
- **MPC 코어** (`src/mpc_longitudinal_controller/mpc_core.py`)
  - 1차 시스템(x_{k+1} = A x_k + B u_k + d) 기반 CVXPY 문제 구성.
  - 입력 변화(delta_u) 변수, jerk penalty(R_delta), jerk 한계(delta_u_max) 파라미터화.
  - 상태 제약 완화를 위한 슬랙 변수(s_lo/s_hi)와 가중치.
  - `update_params` 등을 통해 저속 모드에서 문제 재구성 가능.

### 2.3 상호 연계 및 공통 사항
- 컨트롤 주기 50 Hz 기준(각 패키지에서 `control_rate` 기본값 50).
- 공통 입력: `/Competition_topic` (EgoVehicleStatus), `/planning/global/path`, `/planning/speed_profile/global`, `/imu`.
- Lateral → Longitudinal 인터페이스: `/lateral_mpc/control_info` (steering, current_idx, horizon info) 및 `/lateral_mpc/status/verbose`로 시간 정보 공유.
- 두 패키지 모두 OSQP 튜닝 파라미터를 ROS 파라미터 서버에서 로드하며, Python에서 OSQP 파라미터를 직접 전달.

## 3. 통합 C++ 아키텍처 제안 (`mpc_controller`)

### 3.1 구조 개요
```
Control/mpc_controller/
├── include/mpc_controller/
│   ├── mpc_controller_node.hpp      # 메인 노드 선언
│   ├── lateral_mpc.hpp/.cpp         # 횡방향 MPC (osqpEigen)
│   ├── longitudinal_mpc.hpp/.cpp    # 종방향 MPC (osqpEigen)
│   ├── path_manager.hpp/.cpp        # 경로 캐싱 및 곡률 계산
│   ├── state_estimator.hpp/.cpp     # 상태 추정/센서 융합
│   ├── speed_profile_buffer.hpp/.cpp# 속도 프로파일 업샘플·조회
│   ├── debug_publisher.hpp/.cpp     # 시각화/로그 토픽 발행 유틸
│   └── utils.hpp/.cpp               # 공통 수학/ROS 도우미
├── src/
│   └── mpc_controller_node.cpp
└── docs/
    └── mpc_controller_analysis_plan.md (본 문서)
```

### 3.2 노드 동작 흐름
1. ROS 파라미터 로드 → 횡/종방향 공통 설정 객체(`MpcConfig` 등) 구성.
2. 구독자 등록:
   - `/planning/global/path` → `PathManager::Update(Path)`
   - `/planning/speed_profile/global` → `SpeedProfileBuffer::Update(list<float>)`
   - `/Competition_topic` → Ego 상태 업데이트
   - `/imu` 및 필요 시 추가 센서 → `StateEstimator`
3. 타이머/스레드:
   - 단일 50Hz 타이머에서 순차 실행: 상태 추정 → 경로 최근접점 → 횡방향 MPC → 종방향 MPC.
   - 내부에서 횡방향 결과를 `ControlInfo` 구조체로 보관하고, 종방향 단계가 같은 메모리 참조를 활용.
4. 횡방향 MPC (`LateralMpc`)
   - osqpEigen으로 QP 구성: 시간가변 가중치, 밴드 슬랙, 터미널 앵커링, 입력 변화율 제약을 모두 행렬화.
   - 동적 Np 변경 시 QP 재생성/데이터 리셋: osqpEigen의 `updateHessian`, `updateGradient`, `updateBounds` API 활용.
   - Solver 출력(steering, 미래 δ 시퀀스, 예측 상태)을 `PredictionBundle`로 반환.
5. 종방향 MPC (`LongitudinalMpc`)
   - 1차 시스템 QP: 상태·입력·jerk 제약을 Q, R, R_delta로 구성.
   - 저속 전환 로직: 파라미터 조정 후 필요한 경우 문제 리팩터링.
6. 출력: `/ctrl_cmd` (steering, accel, brake), `/mpc_controller/debug/...` (Marker, Float32MultiArray 등 기존 토픽을 통합 네임스페이스로 유지).
7. 모니터링: `LateralMpcTiming`/커스텀 타이밍 메시지 재사용해 내부 단계 시간 기록.

### 3.3 osqpEigen 적용 포인트
- OSQP 문제 설정을 한 번만 구성하고 반복 실행 시 Hessian/Gradient/Bounds만 업데이트하여 Python 대비 오버헤드 제거.
- 밴드 소프트 제약, IRLS 가중치 등 비선형 보조 로직은 C++에서도 동일하게 구현하되, IRLS 반복 횟수는 기존과 동일(최대 1회)으로 제한.
- 예측 속도 시퀀스(종방향 결과)를 횡방향 시각화에 재사용하기 위해 내부 공유 버퍼 마련.

### 3.4 시각화/디버그 정책
- 기존 RViz Marker 토픽 유지(네임스페이스 포함)하여 시각화 설정 호환성 확보.
- `/lateral_mpc/control_info`는 외부 의존성이 존재할 수 있으므로, 내부 통합 후에도 동일 토픽 발행 옵션을 유지(기본 on/off 파라미터 제공).
- `/mpc_longitudinal/...` 디버그 토픽은 `~debug/` 네임스페이스에서 재발행하거나, 사용하지 않는다면 문서화 후 비활성 옵션 제공.

## 4. 개발 단계 계획
1. **요구사항 확정 및 리소스 준비**
   - osqpEigen, Eigen, ROS message 헤더 정리 및 CMakeLists 업데이트.
   - 기존 Python 파라미터 구조를 C++ 클래스에서 재사용할 수 있도록 YAML 로더/구조체 정의.
2. **핵심 유틸리티 포팅**
   - `PathProcessor` → C++ `PathManager` (곡률 계산, 최근접점, ROI 곡률 추산 함수 포함).
   - `StateEstimator` → C++로 최소 기능(위치, 속도, yaw, yaw rate, ax/ay) 구현. 필요시 TF2 사용.
   - `SpeedProfileBuffer` → 업샘플링·거리 인덱스 변환·stop window 계산.
3. **횡방향 MPC C++화**
   - 상태/입력/슬랙 변수 크기 결정, Hessian/constraint 행렬 구성.
   - 동적 Np 지원: horizon 변경 시 문제 재생성 함수 구현.
   - 밴드, IRLS, state-dependent smoothing 등 부가 기능을 단계적으로 포팅하며 각각 단위 테스트.
4. **종방향 MPC C++화**
   - 기본 QP 구성 → osqpEigen 셋업.
   - 슬랙, 저속 모드 재구성, zero-throttle/stop-window/stopline hold 로직을 노드 측에서 재현.
5. **통합 노드 작성**
   - 순차 실행 루프, 토픽 입출력, 타이밍 계측, 예측 시각화 통합.
   - 기존 메시지(`ControlInfo`, `LateralMpcTiming` 등) 발행 여부를 파라미터화.
6. **검증 및 튜닝**
   - Python 버전과 동일 입력 데이터(rosbag 또는 recorded topics)로 비교 테스트.
   - Solver 결과(steering/accel), 비용, 제약 위반 여부 체크.
   - 타이밍 프로파일 측정(50Hz 유지 확인) 후 파라미터 미세 조정.
7. **문서/런치 갱신**
   - 새 launch 파일 및 README 작성, 기존 Python 패키지 사용 중단 안내.
   - 파라미터 맵핑 표 및 이행 가이드 업데이트.

## 5. 리스크 및 대응
- **QP 행렬 구성 오류**: Python 구현과 비교 검증을 위한 오프라인 스크립트 작성(동일 입력 시 cost/solution diff 검사).
- **동적 Np 재설정 안정성**: osqpEigen에서 `setPrimalInfeasible` 발생 가능 → 재설정 시 warm-start 초기화 및 최소 쿨다운 유지.
- **슬랙/밴드 기능 포팅 난이도**: 초기 버전에서는 필수 기능 우선(핵심 제약 + 기본 가중치) → 이후 선택 기능 단계적 활성.
- **종방향 stop-window 로직**: 거리 기반 계산이 경로 해상도에 민감 → 업샘플링 결과를 검증하고 파라미터 최소/최대값 확인 필요.

## 6. 향후 문서화 계획
- 본 문서를 기반으로 상세 설계 문서(클래스 다이어그램, QP 수식 정리) 추가.
- Python 대비 성능 비교 리포트(rosbag 재생 시 평균 solve time, loop 주기)를 docs에 축적.
- 파라미터 튜닝 가이드(예측 지평, 가중치, 제약) 및 테스트 시나리오 문서 작성 예정.

