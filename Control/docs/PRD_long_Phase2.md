# **Phase 2: MPC 제어기 설계**

## **1. 개요**

본 문서는 `Phase 1`에서 식별된 차량의 종방향 동특성 모델을 기반으로, 목표 속도를 추종하기 위한 모델 예측 제어(MPC) 제어기의 상세 설계안을 정의합니다.

설계의 핵심 목표는 다음과 같습니다.
-   **정확한 속도 추종**: 다양한 주행 시나리오에서 목표 속도를 최소한의 오차로 추종합니다.
-   **안정성 및 제약 조건 준수**: 차량의 물리적 한계(최대 속도, 가감속 능력)를 넘지 않도록 보장합니다.
-   **부드러운 주행 품질**: 급격한 가감속(Jerk)을 억제하여 승차감을 향상시킵니다.
-   **모듈성**: 제어기 핵심 로직을 ROS와 독립적인 Python 클래스로 설계하여 테스트 용이성과 재사용성을 확보합니다.

## **2. 제어기 아키텍처**

제어기는 핵심 MPC 로직을 담당하는 `MpcCore` 클래스와 이를 ROS 환경에서 실행하는 `mpc_node`로 분리됩니다. 본 문서에서는 `MpcCore`의 설계를 다룹니다.

```text
[현재 상태 (속도)] ----> +------------------+
                        |   mpc_node.py    | --(상태/목표)--> [mpc_core.py]
[목표 궤적] ----------> | (Phase 3에서 구현) |
                        |                  | <--(최적 입력)-- [mpc_core.py]
[최적 제어 입력] <------ |                  |
                        +------------------+
```

-   **`mpc_core.py`**:
    -   **역할**: MPC 문제 정식화, 최적화 수행, 결과 반환을 담당하는 순수 Python 클래스입니다.
    -   **입력**: 현재 차량 상태(속도), 예측 구간 동안의 목표 속도 궤적, 제어 파라미터($Q, R, N_p, N_c$).
    -   **출력**: 최적화된 제어 입력 시퀀스 중 첫 번째 값.
    -   **의존성**: `numpy`, `cvxpy` (또는 `casadi`).

## **3. MPC 문제 정식화**

매 제어 주기마다 다음의 최적화 문제를 풀어 최적의 제어 입력을 계산합니다.

### **3.1. 예측 모델 (Prediction Model)**

`Phase 1`에서 식별된 이산 시간 선형 상태 공간 모델을 사용합니다.

$$ 
 x(k+1) = A \cdot x(k) + B \cdot u(k) + d
$$ 

-   $x(k)$: 현재 스텝 $k$에서의 차량 속도 (State)
-   $u(k)$: 현재 스텝 $k$에서의 제어 입력 (Input, `accel - brake`)
-   $A, B, d$: `Phase 1`에서 추정한 시스템 파라미터 (Scalar 값)

### **3.2. 목적 함수 (Cost Function)**

예측 구간($N_p$) 동안의 목표와의 오차와 제어 입력의 크기를 최소화하는 것을 목표로 합니다.

$$ 
 J(k) = \sum_{i=1}^{N_p} \| x(k+i|k) - x_{ref}(k+i) \|^2_Q + \sum_{i=0}^{N_c-1} \| u(k+i|k) \|^2_R
$$ 

-   $x(k+i|k)$: 현재 시점 $k$에서 예측한 $i$ 스텝 후의 속도.
-   $x_{ref}(k+i)$: $i$ 스텝 후의 목표 속도.
-   $u(k+i|k)$: 현재 시점 $k$에서 계산할 $i$ 스텝 후의 제어 입력.
-   $N_p$: **예측 구간 (Prediction Horizon)**. 미래 몇 스텝까지 예측할지를 결정. (e.g., 20 스텝, 100Hz 기준 0.2초)
-   $N_c$: **제어 구간 (Control Horizon)**. 몇 개의 제어 입력을 최적화할지를 결정. ($N_c \le N_p$)
-   $Q$: **상태 가중치 (State Weight)**. 목표 속도 추종 오차에 대한 페널티. 값이 클수록 오차를 줄이려고 노력합니다. (Scalar)
-   $R$: **입력 가중치 (Input Weight)**. 제어 입력의 크기에 대한 페널티. 값이 클수록 제어 입력을 작게 사용하여 부드러운 주행을 유도합니다. (Scalar)

> **주행 품질 향상을 위한 추가 항목 (선택 사항)**:
> 제어 입력의 변화량($\Delta u(k) = u(k) - u(k-1)$)에 대한 페널티 항을 추가하면 급격한 가감속(Jerk)을 효과적으로 억제할 수 있습니다.
> $$ 
> J_{smooth}(k) = J(k) + \sum_{i=0}^{N_c-1} \| \Delta u(k+i|k) \|^2_{R_{\Delta}}
> $$ 
> 이 경우, 최적화 변수에 $\Delta u$가 추가되어야 합니다.

### **3.3. 제약 조건 (Constraints)**

최적화 문제의 해가 물리적으로 유의미한 값을 갖도록 제한합니다.

1.  **시스템 모델 제약**:
    -   $x(k+i+1|k) = A \cdot x(k+i|k) + B \cdot u(k+i|k) + d$  for $i = 0, ..., N_p-1$

2.  **상태 변수 제약 (State Constraints)**:
    -   $v_{min} \le x(k+i|k) \le v_{max}$ (e.g., $0 \text{ km/h} \le x \le 120 \text{ km/h}$)

3.  **제어 입력 제약 (Input Constraints)**:
    -   $u_{min} \le u(k+i|k) \le u_{max}$ (e.g., MORAI 시뮬레이터 기준 `-1.0` (최대 제동) ~ `1.0` (최대 가속))

## **4. 최적화 솔버 (QP Solver)**

위에서 정식화된 문제는 **제약 조건이 있는 선형 이차 계획법 (Constrained Linear Quadratic Programming, QP)** 문제입니다. Python 기반의 최적화 라이브러리를 사용하여 이 문제를 해결합니다.

-   **솔버 선택**: `CVXPY`
    -   **장점**: 문제 정식화를 수학적 표현과 거의 동일하게 Python 코드로 작성할 수 있어 가독성이 매우 높고 사용이 직관적입니다. OSQP, ECOS 등 다양한 백엔드 QP 솔버를 지원합니다.
    -   **단점**: `casadi`에 비해 코드 생성 및 실행 속도가 다소 느릴 수 있으나, 본 프로젝트의 제어 주기(10~20ms) 내에서는 충분한 성능을 보입니다.

-   **`CVXPY` 구현 예시**:
    ```python
    import cvxpy as cp
    import numpy as np

    # MPC 파라미터 정의
    Np = 20  # Prediction Horizon
    Nc = 5   # Control Horizon
    Q = 1.0
    R = 0.5

    # 최적화 변수 정의
    u = cp.Variable((Nc, 1))
    x = cp.Variable((Np + 1, 1))

    # 목적 함수 및 제약 조건 리스트 초기화
    objective = 0
    constraints = [x[0] == current_velocity] # 초기 조건

    # 루프를 돌며 목적 함수와 제약 조건 구성
    for k in range(Np):
        # ... (목적 함수 항 추가)
        objective += Q * cp.sum_squares(x[k+1] - x_ref[k])

        # ... (시스템 모델 제약 조건 추가)
        constraints += [x[k+1] == A * x[k] + B * u[k] + d] # u는 Nc까지만 유효

    for k in range(Nc):
        objective += R * cp.sum_squares(u[k])

    # ... (상태 및 입력 제약 조건 추가)
    constraints += [
        v_min <= x,
        x <= v_max,
        u_min <= u,
        u <= u_max
    ]

    # 문제 정의 및 풀이
    problem = cp.Problem(cp.Minimize(objective), constraints)
    problem.solve(solver=cp.OSQP, warm_start=True)

    # 결과 반환
    optimal_input = u.value[0, 0]
    ```

## **5. `MpcCore` 클래스 설계**

```python
# mpc_core.py

import cvxpy as cp
import numpy as np

class MpcCore:
    def __init__(self, A, B, d, params):
        """
        MPC 제어기 초기화
        :param A, B, d: 시스템 모델 파라미터
        :param params: Np, Nc, Q, R, v_max, u_min, u_max 등 제어 파라미터 딕셔너리
        """
        self.A, self.B, self.d = A, B, d
        self.params = params
        self.problem = None # CVXPY Problem 객체
        self.u_optimal = None # 최적화 변수

        self._setup_problem()

    def _setup_problem(self):
        """
        CVXPY를 사용하여 최적화 문제를 한 번만 설정합니다.
        이후에는 파라미터 값만 업데이트하여 반복적으로 풉니다.
        """
        # CVXPY Parameter 객체들을 사용하여 실시간으로 업데이트될 값들을 정의
        self.x0 = cp.Parameter((1, 1), name="initial_state")
        self.x_ref = cp.Parameter((self.params['Np'], 1), name="reference_trajectory")

        # 여기에 CVXPY 문제 정식화 코드를 작성 (위 예시 참조)
        # ...

        # problem.solve()를 제외한 문제 정의까지 완료
        self.problem = cp.Problem(...)

    def solve(self, current_velocity, target_velocity_trajectory):
        """
        현재 상태와 목표 궤적을 받아 최적화 문제를 풉니다.
        :param current_velocity: 현재 차량 속도
        :param target_velocity_trajectory: 예측 구간 동안의 목표 속도 배열 (길이 Np)
        :return: 최적 제어 입력 (u*)
        """
        # CVXPY Parameter 값 업데이트
        self.x0.value = np.array([[current_velocity]])
        self.x_ref.value = target_velocity_trajectory.reshape(-1, 1)

        # 문제 풀이
        self.problem.solve(solver=cp.OSQP, warm_start=True)

        if self.problem.status not in ["optimal", "optimal_inaccurate"]:
            print("Warning: MPC problem could not be solved!")
            return 0.0 # 안전한 값 반환

        # 계산된 제어 입력 시퀀스 중 첫 번째 값 반환
        return self.u_optimal.value[0, 0]

```

## **6. 결론 및 다음 단계**

본 설계안에 따라 `mpc_core.py` 모듈을 구현하면, ROS와 독립적으로 테스트 및 검증이 가능한 MPC 제어기 핵심 로직이 완성됩니다.

**다음 단계**:
-   `Phase 3: ROS 노드 구현 및 통합`에서는 본 문서에서 설계한 `MpcCore` 클래스를 임포트하여 실제 ROS 환경과 연동하는 `mpc_node`를 개발합니다. 이 노드는 센서 데이터로부터 현재 상태를 추정하고, 목표 속도를 생성하여 `MpcCore`에 전달하며, 반환된 제어 입력을 차량에 맞게 변환하여 발행하는 역할을 수행합니다.