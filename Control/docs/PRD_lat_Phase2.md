# **Phase 2: Lateral MPC 제어기 설계 및 구현**

## **1. 개요**

본 문서는 `Phase 1`에서 식별된 차량의 횡방향 동역학 모델을 기반으로, 경로 추종을 위한 Lateral MPC 제어기의 상세 설계 및 구현 계획을 정의합니다.

Pure Pursuit의 언더스티어 문제를 해결하고, 50km/h 이하 속도 영역에서 정밀한 경로 추종 성능을 달성하는 것이 핵심 목표입니다.

**설계 핵심 목표**:
- **정밀한 경로 추종**: 횡방향 오차 < 0.2m, 헤딩 오차 < 5°
- **언더스티어 보상**: 동역학 모델 기반 예측으로 Pure Pursuit 대비 30% 개선
- **부드러운 조향**: 급격한 조향 변화 억제로 승차감 향상
- **실시간 성능**: 50Hz 제어 주기 내 최적화 문제 해결 (< 20ms)

## **2. 제어기 아키텍처**

### **2.1. 전체 시스템 구조**

```text
[경로 정보] ---------> +----------------------+
                      |   lateral_mpc_node   | ---> [LateralMpcCore]
[차량 상태] ---------> |    (Phase 3)         |      (순수 Python)
                      |                      | <--- [최적 조향각]
[최적 조향 명령] <---- |                      |
                      +----------------------+

LateralMpcCore 내부:
- Error Dynamics Model 기반 예측
- QP 최적화 문제 정식화 및 해결
- 곡률 기반 Feedforward 조향
```

### **2.2. 제어 모드 선택**

```python
class ControlMode(Enum):
    KINEMATIC = "kinematic"    # 빠른 프로토타이핑용
    DYNAMIC = "dynamic"        # 고성능 제어용
```

## **3. MPC 문제 정식화**

### **3.1. Error Dynamics Model (동역학 모델)**

경로 추종 오차를 직접적으로 다루는 Error State 기반 모델:

**상태 벡터**:
$$\mathbf{x}_e = [e_y, \dot{e}_y, e_\psi, \dot{e}_\psi]^T$$

- $e_y$: 횡방향 위치 오차 [m]
- $\dot{e}_y$: 횡방향 속도 오차 [m/s]
- $e_\psi$: 헤딩 오차 [rad]
- $\dot{e}_\psi$: 헤딩 변화율 오차 [rad/s]

**연속시간 상태공간 모델**:
$$\dot{\mathbf{x}}_e = \mathbf{A}_e\mathbf{x}_e + \mathbf{B}_u\mathbf{u} + \mathbf{B}_{cr}C_r$$

여기서 시스템 행렬은:

$$\mathbf{A}_e = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{C_{\alpha f} + C_{\alpha r}}{m v_x} & \frac{C_{\alpha f} + C_{\alpha r}}{m} & \frac{-l_f C_{\alpha f} + l_r C_{\alpha r}}{m v_x} \\
0 & 0 & 0 & 1 \\
0 & \frac{-l_f C_{\alpha f} + l_r C_{\alpha r}}{I_z v_x} & \frac{l_f C_{\alpha f} - l_r C_{\alpha r}}{I_z} & -\frac{l_f^2 C_{\alpha f} + l_r^2 C_{\alpha r}}{I_z v_x}
\end{bmatrix}$$

$$\mathbf{B}_u = \begin{bmatrix} 0 \\ \frac{C_{\alpha f}}{m} \\ 0 \\ \frac{l_f C_{\alpha f}}{I_z} \end{bmatrix}, \quad
\mathbf{B}_{cr} = \begin{bmatrix} 0 \\ - v_x^2 \\ 0 \\ 0 \end{bmatrix}$$

### **3.2. Kinematic Model (기구학 모델) - 빠른 구현용**

단순화된 모델로 초기 프로토타이핑:

**상태 벡터**: $\mathbf{x}_k = [e_y, e_\psi]^T$

**이산시간 모델**:
$$\mathbf{x}_{k+1} = \begin{bmatrix} 
1 & v_x T_s \\
0 & 1 
\end{bmatrix} \mathbf{x}_k + \begin{bmatrix}
0 \\ \frac{v_x T_s}{L}
\end{bmatrix} \delta_f$$

### **3.3. 목적 함수**

예측 구간 동안의 추종 오차와 제어 입력을 최소화:

$$J = \sum_{k=0}^{N_p-1} \left( \mathbf{x}_e^T(k) \mathbf{Q} \mathbf{x}_e(k) + u^T(k) R u(k) + \Delta u^T(k) R_\Delta \Delta u(k) \right) + \mathbf{x}_e^T(N_p) \mathbf{P} \mathbf{x}_e(N_p)$$

**가중치 행렬 설계**:
```python
Q = np.diag([100, 10, 50, 5])    # [e_y, ė_y, e_ψ, ė_ψ]에 대한 가중치
R = 1.0                           # 조향 입력 크기 페널티
R_delta = 10.0                    # 조향 변화량 페널티 (승차감)
P = Q * 10                        # 종단 상태 가중치
```

### **3.4. 제약 조건**

```python
# 1. 시스템 동역학 제약
x[k+1] = A_d @ x[k] + B_d @ u[k] + B_cr_d @ Cr[k]

# 2. 조향각 제약
delta_min = -40 * np.pi/180  # -40도
delta_max = 40 * np.pi/180   # 40도

# 3. 조향 변화율 제약 (actuator limitation)
delta_rate_max = 30 * np.pi/180  # 30 deg/s at 50Hz = 0.6 deg/step

# 4. 경로 이탈 방지 (선택적)
e_y_max = 2.0  # 최대 횡방향 오차 2m
```

## **4. Feedforward 조향 설계**

### **4.1. 곡률 기반 Feedforward**

정상상태 조향각을 초기값으로 제공하여 수렴 속도 향상:

```python
def calculate_feedforward_steering(curvature, v_x, model_params):
    """
    경로 곡률과 언더스티어 특성을 고려한 feedforward 조향각
    """
    L = model_params['wheelbase']
    
    if model_params['mode'] == 'kinematic':
        # Ackermann 조향
        delta_ff = np.arctan(L * curvature)
    else:
        # 언더스티어 보상 포함
        K_v = model_params['understeer_gradient']
        delta_ff = L * curvature + K_v * v_x**2 * curvature
    
    return delta_ff
```

### **4.2. 예견 거리 기반 곡률 계산**

```python
def get_preview_curvature(path, current_pose, preview_distance):
    """
    예견 거리 앞의 경로 곡률 계산
    """
    preview_point = get_point_at_distance(path, current_pose, preview_distance)
    
    # 3점을 이용한 곡률 계산
    p1 = get_point_at_distance(path, current_pose, preview_distance - 2)
    p2 = preview_point
    p3 = get_point_at_distance(path, current_pose, preview_distance + 2)
    
    curvature = calculate_curvature_from_points(p1, p2, p3)
    return curvature
```

## **5. LateralMpcCore 클래스 설계**

```python
# lateral_mpc_core.py

import numpy as np
import cvxpy as cp
from enum import Enum

class LateralMpcCore:
    def __init__(self, model_params, control_params):
        """
        Lateral MPC 제어기 초기화
        
        Args:
            model_params: 모델 파라미터 (L, m, Iz, Caf, Car 등)
            control_params: 제어 파라미터 (Np, Nc, Q, R 등)
        """
        self.model_params = model_params
        self.control_params = control_params
        
        # 제어 모드 설정
        self.mode = model_params.get('mode', 'kinematic')
        
        # 최적화 문제 설정
        self._setup_optimization_problem()
        
    def _setup_optimization_problem(self):
        """CVXPY 최적화 문제 한 번만 설정"""
        Np = self.control_params['Np']
        Nc = self.control_params['Nc']
        
        # 최적화 변수
        if self.mode == 'kinematic':
            nx = 2  # [e_y, e_psi]
        else:
            nx = 4  # [e_y, ė_y, e_psi, ė_psi]
            
        self.x = cp.Variable((nx, Np + 1))
        self.u = cp.Variable((1, Nc))
        self.delta_u = cp.Variable((1, Nc))
        
        # 파라미터 (실시간 업데이트)
        self.x0 = cp.Parameter(nx)
        self.x_ref = cp.Parameter((nx, Np + 1))
        self.u_prev = cp.Parameter()
        self.curvature = cp.Parameter(Np)
        
        # 목적 함수
        Q = self.control_params['Q']
        R = self.control_params['R']
        R_delta = self.control_params['R_delta']
        
        cost = 0
        for k in range(Np):
            cost += cp.quad_form(self.x[:, k] - self.x_ref[:, k], Q)
        for k in range(Nc):
            cost += R * cp.square(self.u[0, k])
            cost += R_delta * cp.square(self.delta_u[0, k])
            
        # 제약 조건
        constraints = []
        
        # 초기 조건
        constraints.append(self.x[:, 0] == self.x0)
        
        # 시스템 동역학
        A_d, B_d = self._get_discrete_model()
        for k in range(Np):
            if k < Nc:
                u_k = self.u[0, k]
            else:
                u_k = self.u[0, Nc-1]  # 마지막 제어 입력 유지
                
            constraints.append(
                self.x[:, k+1] == A_d @ self.x[:, k] + B_d * u_k
            )
        
        # 조향각 제약
        delta_limits = self.control_params['delta_limits']
        constraints.append(self.u >= delta_limits[0])
        constraints.append(self.u <= delta_limits[1])
        
        # 조향 변화율 제약
        constraints.append(self.delta_u[0, 0] == self.u[0, 0] - self.u_prev)
        for k in range(1, Nc):
            constraints.append(self.delta_u[0, k] == self.u[0, k] - self.u[0, k-1])
            
        delta_rate_max = self.control_params['delta_rate_max']
        constraints.append(cp.abs(self.delta_u) <= delta_rate_max)
        
        # 문제 정의
        self.problem = cp.Problem(cp.Minimize(cost), constraints)
        
    def _get_discrete_model(self):
        """이산시간 모델 행렬 반환"""
        Ts = self.control_params['Ts']
        
        if self.mode == 'kinematic':
            # Kinematic bicycle model
            v_x = self.model_params['v_x']
            L = self.model_params['L']
            
            A_d = np.array([[1, v_x * Ts],
                           [0, 1]])
            B_d = np.array([[0],
                           [v_x * Ts / L]])
                           
        else:
            # Dynamic bicycle model
            # Zero-order hold discretization
            A_c = self._get_continuous_A_matrix()
            B_c = self._get_continuous_B_matrix()
            
            # 간단한 오일러 근사 (더 정확한 방법은 scipy.signal.cont2discrete 사용)
            A_d = np.eye(4) + A_c * Ts
            B_d = B_c * Ts
            
        return A_d, B_d
    
    def solve(self, current_state, reference_path, current_speed):
        """
        MPC 최적화 문제 해결
        
        Args:
            current_state: 현재 오차 상태 [e_y, (ė_y), e_psi, (ė_psi)]
            reference_path: 참조 경로 정보
            current_speed: 현재 차속
            
        Returns:
            optimal_steering: 최적 조향각 [rad]
            predicted_trajectory: 예측 궤적
        """
        # 모델 파라미터 업데이트 (속도 의존적)
        self.model_params['v_x'] = current_speed
        
        # 파라미터 값 업데이트
        self.x0.value = current_state
        self.x_ref.value = np.zeros_like(self.x.value)  # 오차는 0이 목표
        
        # Feedforward 조향각 계산
        curvatures = self._calculate_path_curvatures(reference_path)
        self.curvature.value = curvatures
        
        # 이전 제어 입력 (warm start)
        if hasattr(self, 'last_u'):
            self.u_prev.value = self.last_u
        else:
            self.u_prev.value = 0.0
            
        # 최적화 문제 해결
        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
            
            if self.problem.status not in ["optimal", "optimal_inaccurate"]:
                print(f"Warning: MPC solver status: {self.problem.status}")
                return self.u_prev.value, None
                
        except Exception as e:
            print(f"MPC solver failed: {e}")
            return self.u_prev.value, None
            
        # 최적 제어 입력 추출
        optimal_steering = self.u.value[0, 0]
        self.last_u = optimal_steering
        
        # 예측 궤적 (디버깅용)
        predicted_trajectory = self.x.value
        
        return optimal_steering, predicted_trajectory
    
    def _calculate_path_curvatures(self, reference_path):
        """참조 경로의 곡률 계산"""
        Np = self.control_params['Np']
        preview_dist = self.control_params.get('preview_distance', 2.0)
        
        curvatures = np.zeros(Np)
        for k in range(Np):
            # k 스텝 앞의 예상 위치에서의 곡률
            lookahead = preview_dist + k * self.control_params['Ts'] * self.model_params['v_x']
            curvatures[k] = self._get_curvature_at_distance(reference_path, lookahead)
            
        return curvatures
```

## **6. 제어 파라미터 튜닝 가이드**

### **6.1. 초기 파라미터 설정**

```python
control_params = {
    # MPC horizons
    'Np': 20,           # Prediction horizon (20 steps = 0.4s @ 50Hz)
    'Nc': 5,            # Control horizon
    'Ts': 0.02,         # Sample time (50Hz)
    
    # Weights
    'Q': np.diag([100, 10, 50, 5]),  # State error weights
    'R': 1.0,                         # Control effort weight
    'R_delta': 10.0,                  # Control rate weight
    
    # Constraints
    'delta_limits': [-0.7, 0.7],      # ±40 degrees in radians
    'delta_rate_max': 0.01,           # 30 deg/s at 50Hz
    
    # Preview
    'preview_distance': 5.0,          # meters
}
```

### **6.2. 튜닝 전략**

| 문제 현상 | 튜닝 방향 | 파라미터 조정 |
|----------|----------|--------------|
| 횡방향 오차 큼 | 추종 성능 강화 | Q[0,0] ↑ |
| 헤딩 오차 큼 | 방향 정렬 강화 | Q[2,2] ↑ |
| 조향 떨림 | 부드러운 제어 | R, R_delta ↑ |
| 느린 응답 | 빠른 응답 | R ↓, Nc ↑ |
| 코너 언더스티어 | 예견 거리 증가 | preview_distance ↑ |

## **7. 성능 검증 메트릭**

```python
class PerformanceMetrics:
    def __init__(self):
        self.lateral_errors = []
        self.heading_errors = []
        self.steering_commands = []
        
    def update(self, e_y, e_psi, delta):
        self.lateral_errors.append(e_y)
        self.heading_errors.append(e_psi)
        self.steering_commands.append(delta)
        
    def compute_statistics(self):
        return {
            'lateral_rmse': np.sqrt(np.mean(np.square(self.lateral_errors))),
            'lateral_max': np.max(np.abs(self.lateral_errors)),
            'heading_rmse': np.sqrt(np.mean(np.square(self.heading_errors))),
            'steering_smoothness': np.std(np.diff(self.steering_commands)),
        }
```

## **8. 구현 우선순위 및 일정**

### **Phase 2A: Kinematic Model 구현 (Week 1)**
1. `LateralMpcCore` 클래스 기본 구조
2. Kinematic model 기반 예측
3. 기본 QP 최적화 구현
4. 단위 테스트 작성

### **Phase 2B: Dynamic Model 업그레이드 (Week 2)**
1. Dynamic model 행렬 구현
2. 파라미터 로딩 (Phase 1 결과)
3. Feedforward 조향 추가
4. 모델 전환 로직

### **Phase 2C: 성능 최적화 (Week 3)**
1. 실시간 성능 프로파일링
2. OSQP 솔버 파라미터 최적화
3. Warm-start 구현
4. 병렬화 검토

## **9. 다음 단계 (Phase 3 연계)**

Phase 2에서 구현된 `LateralMpcCore`는 Phase 3에서:
- ROS 노드로 래핑되어 실제 시스템과 통합
- 경로 정보 토픽 구독 및 처리
- 차량 상태 추정기와 연동
- `/ctrl_cmd` 발행을 통한 차량 제어

## **10. 예상 성능 목표**

| 메트릭 | Pure Pursuit | Lateral MPC (목표) | 개선율 |
|--------|--------------|-------------------|--------|
| 횡방향 RMSE | 0.5m | 0.2m | 60% |
| 최대 오차 | 1.2m | 0.5m | 58% |
| 언더스티어 (40km/h, R=30m) | 15° | 10° | 33% |
| 계산 시간 | < 1ms | < 20ms | - |