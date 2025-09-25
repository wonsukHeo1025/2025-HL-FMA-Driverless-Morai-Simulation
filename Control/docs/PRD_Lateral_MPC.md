# **1단계: MPC를 위한 차량 모델 선택**

어떤 모델을 선택하느냐에 따라 필요한 파라미터와 제어 성능이 달라집니다. 사용자의 목표(50km/h 이하, Pure Pursuit의 언더스티어 개선)를 고려할 때 두 가지 모델 모두 장단점이 있습니다.

### **선택지 1: Kinematic Bicycle Model (기구학 모델)**

- **개념:** 차량의 힘을 고려하지 않고, 조향각과 속도에 따른 기하학적 움직임만으로 차량의 거동을 예측합니다. 사용자가 정리한 최종 상태 공간 모델이 바로 이것입니다.
- **장점:**
    - **단순함:** 필요한 파라미터가 휠베이스(`L`) 하나뿐이라 모델 구축이 매우 쉽습니다.
    - **직관성:** 모델의 동작을 이해하기 쉽고, 구현이 간단합니다.
    - **낮은 계산 비용:** MPC 연산량이 적어 실시간성이 보장됩니다.
    - **저속 성능:** 타이어 슬립이 적은 저속(약 40km/h 이하)에서는 충분히 좋은 성능을 보입니다.
- **단점:**
    - **물리 현상 무시:** 타이어 슬립, 횡력, 관성 등을 고려하지 않아 차량의 동역학적 특성(언더스티어, 오버스티어)을 직접적으로 모델링할 수 없습니다.
    - **고속/고횡가속 한계:** 속도가 높아지거나 급격한 코너링 시 실제 차량 거동과의 오차가 커집니다.
- **추천 대상:** MPC를 처음 구현해보거나, 빠른 프로토타이핑을 통해 전체 시스템을 먼저 구축하고 싶을 때 매우 좋은 출발점입니다.

### **선택지 2: Dynamic Bicycle Model (동역학 모델)**

- **개념:** 타이어에서 발생하는 횡력과 차량의 관성을 고려하여 차량의 거동을 예측합니다. 사용자가 정리한 'Error Dynamics Model'이 이 모델을 기반으로 합니다.
- **장점:**
    - **높은 정확도:** 물리 현상을 반영하므로 더 넓은 속도 영역에서 실제 차량과 유사한 거동을 예측합니다.
    - **언더스티어/오버스티어 모델링:** 타이어 코너링 강성($C_{\alpha f}, C_{\alpha r}$)을 통해 차량의 핸들링 특성을 직접 모델에 반영할 수 있어, Pure Pursuit의 단점을 개선하는 데 더 효과적입니다.
    - **고성능 제어:** 정확한 예측을 바탕으로 더 공격적이고 안정적인 제어가 가능합니다.
- **단점:**
    - **복잡성:** 차량 질량($m$), 요 관성 모멘트($I_z$), 코너링 강성($C_{\alpha f}, C_{\alpha r}$) 등 식별해야 할 파라미터가 많고 어렵습니다.
    - **높은 계산 비용:** 모델이 복잡한 만큼 MPC의 연산 부담이 커질 수 있습니다.
- **추천 대상:** 기존 제어기(Pure Pursuit)의 성능 한계를 명확히 개선하고 싶고, 보다 정밀한 제어를 목표로 할 때 적합합니다.

---

### Dynamic Bicycle Model 핵심 정리 (가정·유도·상태공간)

아래 정리는 동역학 자전거 모델을 MPC에 적용하기 위해 필요한 핵심 수식과 활용 포인트를 요약합니다. 모델은 소각 조향 및 일정 종속도 구간에서의 선형 근사를 전제로 합니다.

- **가정**
  - **작은 조향각** $\delta_f$, **일정 종방향 속도** $v_x$ 구간 가정
  - 차량은 XY 평면 강체, 타이어는 선형 코너링 강성 모델 사용
  - 단기 제어 샘플 구간에서는 종력 무시 ⇒ NO longitudinal force within each dt

- **힘 평형 (y 방향)**
  - $ m(\dot v_y + v_x \dot{\psi}) = F_r + F_f $
  - $ \dot v_y + v_x \dot{\psi} = \tfrac{1}{m}(F_r + F_f) $

- **모멘트 평형 (yaw)**
  - $ I_z \dot\omega = -l_r F_r + l_f F_f $
  - $ \ddot\psi = \tfrac{1}{I_z}(l_f F_f - l_r F_r) $

- **선형 타이어 모델**
  - $ F_r = -C_{\alpha r}\,\alpha_r \approx -C_{\alpha r}\,\tfrac{v_y - \omega l_r}{v_x} $
  - $ F_f = -C_{\alpha f}\,\alpha_f \approx C_{\alpha f}\Big(\delta_f - \tfrac{v_y + \omega l_f}{v_x}\Big) $

- **연속시간 상태공간 (기본 상태 선택: $\mathbf{x}=[y,\ v_y,\ \psi,\ \omega]^T$, 입력 $\mathbf{u}=[\delta_f]$)**

$$
\frac{d}{dt}
\begin{bmatrix}
 y \\
 \dot{y} \\
 \psi \\
 \dot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
 0 & 1 & 0 & 0 \\
 0 & -\tfrac{C_{\alpha f}+C_{\alpha r}}{m v_x} & 0 & \tfrac{-l_f C_{\alpha f}+l_r C_{\alpha r}}{m v_x} - v_x \\
 0 & 0 & 0 & 1 \\
 0 & \tfrac{-l_f C_{\alpha f}+l_r C_{\alpha r}}{I_z v_x} & 0 & -\tfrac{l_f^2 C_{\alpha f}+l_r^2 C_{\alpha r}}{I_z v_x}
\end{bmatrix}
\begin{bmatrix}
 y \\
 \dot{y} \\
 \psi \\
 \dot{\psi}
\end{bmatrix}
 +
\begin{bmatrix}
 0 \\
 \tfrac{C_{\alpha f}}{m} \\
 0 \\
 \tfrac{l_f C_{\alpha f}}{I_z}
\end{bmatrix}
\,[\delta_f]
$$

- **필요 파라미터**: $m,\ I_z,\ l_f,\ l_r,\ C_{\alpha f},\ C_{\alpha r}$; 실시간 필요: $v_x$

- **에러 동역학과의 연결**: 경로 곡률 $\kappa=1/R$를 외란(Feedforward)으로 취급하고, $e_y, e_\psi$ 기반 상태로 선형화·좌표변환하면, 아래 3단계에 정리된 에러 동역학 형태로 귀결됩니다.

### 정상상태 선회·언더스티어 그래디언트 $K_v$

- **정상상태 조향각**: $ F_f+F_r = m\,\tfrac{v_x^2}{R},\ \ F_f l_f - F_r l_r = 0 \Rightarrow \delta = \tfrac{L}{R} + K_v\,\tfrac{v_x^2}{R} $
- **언더스티어 그래디언트**: $ K_v = \tfrac{m}{L}\Big(\tfrac{l_r}{C_{\alpha f}} - \tfrac{l_f}{C_{\alpha r}}\Big) $
  - $K_v>0$: 언더스티어, $K_v<0$: 오버스티어, $K_v=0$: 뉴트럴
  - 파라미터 검증에 매우 유용. 실험 데이터로 구한 \(K_v\)가 모델 기반 계산과 일치하는지 확인

- **특성속도/임계속도**
  - $ v_{char} = \sqrt{\tfrac{L}{K_v}} $  (언더스티어 차량에서 정의)
  - $ v_{crit} = \sqrt{\tfrac{L}{-K_v}} $  (오버스티어 차량에서 정의)

- **정상상태 요레이트 레퍼런스**
  - $ \dot\psi_{des} = \tfrac{v_x}{R} = \tfrac{v_x}{1+(v_x/v_{char})^2}\,\tfrac{\delta_f}{L} $
  - MPC의 참조 신호로 활용 가능. 곡률 기반 피드포워드 초기화에 특히 유용

⇒ 실무 팁: 곡률 $\kappa$가 주어지면 $\delta_{ff} = L\,\kappa + K_v\,v_x^2\,\kappa$를 초기 조향으로 제공하고, MPC가 미세 조정을 수행하도록 설정하면 수렴 속도와 안정성이 향상됩니다.

---

### **2단계: 데이터 수집 및 파라미터 식별**

이 단계가 MPC 성능을 좌우하는 가장 중요한 과정입니다. 어떤 모델을 선택했는지에 따라 필요한 데이터와 분석 방법이 달라집니다.

### **수집해야 할 데이터 목록**

어떤 방법을 사용하든 아래 데이터들은 기본적으로 수집해야 합니다. MORAI 시뮬레이터에서 제공하는 Ground Truth 값과 센서 데이터를 활용합니다.

- **제어 입력 (Inputs):**
    - `Steering Angle (δ)`: 제어기가 차량에 보낸 최종 조향각 (단위: rad)
- **차량 상태 (States):**
    - `Longitudinal Velocity (v_x)`: 차량의 종방향 속도 (m/s)
    - `Lateral Velocity (v_y)`: 차량의 횡방향 속도 (m/s)
    - `Yaw Rate (ψ_dot)`: 차량의 요 레이트 (rad/s)
    - `Lateral Acceleration (a_y)`: IMU 센서에서 얻는 횡방향 가속도 (m/s²)

### **데이터 수집을 위한 주행 시나리오 (How to Collect?)**

"타각 주고 빙빙 돌기"는 좋은 방법 중 하나입니다. 목적에 따라 다양한 시나리오를 수행하여 풍부한 데이터를 얻는 것이 중요합니다. 종방향 속도는 목표 속도(40~50km/h) 근처에서 일정하게 유지하며 테스트합니다.

1. **Steady-State Cornering (일정 선회 주행):**
    - **방법:** 일정한 조향각을 유지하며 차량이 안정적인 원을 그리며 돌 때까지 주행합니다. 다양한 조향각과 속도 조합으로 여러 번 반복합니다. (예: 20km/h에서 5도, 10도, 40km/h에서 5도, 10도)
    - **목적:** 차량의 정상상태(Steady-state) 특성을 파악하는 데 가장 좋습니다. 사용자가 정리한 언더스티어 그래디언트($K_v$)를 실험적으로 구하는 데 사용됩니다.
2. **Step Steer (스텝 조향 입력):**
    - **방법:** 일정한 속도로 직진하다가, 가능한 한 빠르게 특정 조향각을 입력하고 그 각도를 유지합니다. 차량이 새로운 방향으로 안정화될 때까지의 과도응답(Transient response)을 기록합니다.
    - **목적:** 차량의 **과도응답 특성**을 파악하는 데 핵심적입니다. 요 관성 모멘트($I_z$)와 같이 시스템의 반응 속도와 관련된 파라미터를 식별하는 데 매우 유용한 데이터를 제공합니다.
3. **Sine Sweep (정현파 조향 입력):**
    - **방법:** 일정한 속도로 직진하며, 낮은 주파수에서 높은 주파수까지 서서히 변화하는 정현파(sine wave) 형태의 조향 입력을 가합니다.
    - **목적:** 시스템의 주파수 응답 특성을 얻을 수 있어, 보다 전문적인 시스템 식별(System Identification) 기법에 사용하기 가장 좋은 데이터입니다.

### **파라미터 식별 방법론 (How to Build the Model?)**

수집한 데이터를 이용해 모델의 파라미터를 찾는 과정입니다.

- **Kinematic Model의 경우:**
    - `L` (휠베이스) 값은 MORAI 시뮬레이터의 차량 제원(Specification)에서 보통 제공됩니다. 찾아서 그대로 사용하면 됩니다.
- **Dynamic Model의 경우 ($m, I_z, l_f, l_r, C_{\alpha f}, C_{\alpha r}$):**
    - `m, l_f, l_r`: 휠베이스와 마찬가지로 차량 제원에서 제공될 가능성이 높습니다.
    - `I_z, C_{\\alpha f}, C_{\\alpha r}`: 이 값들은 보통 직접 찾아야 합니다. 이것이 핵심입니다.
    
    **방법 1: 분석적/그래프 기반 추정 (정상상태 데이터 활용)**
    
    1. 정상상태 선회 데이터로부터 각 실험의 횡가속도($a_y = v_x \cdot \dot{\psi}$)와 조향각($\delta_f$)을 얻습니다.
    2. X축을 횡가속도($a_y$), Y축을 조향각($\delta_f$)으로 하는 그래프를 그립니다.
    3. 데이터 포인트들을 선형 회귀(Linear Regression)하여 직선의 방정식을 구합니다.
    4. 이 직선의 기울기가 바로 언더스티어 그래디언트($K_v$)가 됩니다.
    5. 사용자가 정리한 식 $\delta = \frac{L}{R} + K_v \frac{v^2_x}{R}$ 에서 $a_y = v_x^2/R$ 이므로, $\delta = \frac{L}{v_x^2}a_y + K_v a_y$ 와 유사한 형태로, 기울기는 $K_v$와 관련이 있습니다. (정확히는 $\delta$ vs $a_y$ 그래프의 기울기가 $K_v$ 입니다.)
    6. $K_v = \frac{m}{L}(\frac{l_r}{C_{\alpha f}}-\frac{l_f}{C_{\alpha r}})$ 이므로, $K_v$ 값을 알면 $C_{\alpha f}$와 $C_{\alpha r}$ 사이의 관계식 하나를 얻을 수 있습니다. 이것만으로는 두 값을 정확히 분리할 수 없지만, 모델 검증에 매우 유용합니다.
    
    **방법 2: 시스템 식별을 통한 수치적 최적화 (과도응답 데이터 활용)**
    
    - **개념:** Dynamic Bicycle Model의 상태 공간 방정식에 `m, lf, lr` 등 알고 있는 값을 넣고, 모르는 파라미터($I_z, C_{\alpha f}, C_{\alpha r}$)는 변수로 둡니다. 이후, 수치 최적화 알고리즘을 사용하여 **모델의 예측값**(예측된 $\dot{\psi}, a_y$)이 **실제 수집한 데이터**와 가장 유사해지는 파라미터 조합을 찾는 방법입니다.
    - **절차:**
        1. Step Steer 또는 Sine Sweep 데이터를 준비합니다.
        2. Python의 `scipy.optimize.least_squares`와 같은 라이브러리를 사용합니다.
        3. Cost 함수를 "실제 측정된 요레이트 - 모델이 예측한 요레이트"의 제곱의 합(Sum of Squared Errors)으로 정의합니다.
        4. 최적화 알고리즘을 실행하여 이 Cost를 최소화하는 $I_z, C_{\alpha f}, C_{\alpha r}$를 찾습니다.
    - **장점:** 여러 파라미터를 동시에 추정할 수 있으며, 가장 정확하고 신뢰도 높은 결과를 제공합니다. **가장 추천하는 방법입니다.**

---

### **3단계: MPC 제어기 설계 및 구현**

모델과 파라미터가 준비되었다면, 이를 바탕으로 MPC 제어기를 구성합니다. 사용자가 정리한 **Error Dynamics Model**은 MPC 설계에 매우 이상적인 형태입니다.

1. **상태(State) 및 입력(Input) 정의:**
    - 사용자가 정의한 Error State Vector($\mathbf{x}_{e} = [e_y, \dot{e_y}, e_\psi, \dot{e_\psi}]^T$)는 훌륭한 선택입니다. 경로 추종에 필요한 모든 정보를 담고 있습니다.
    - 제어 입력은 조향각($\mathbf{u} = [\delta_f]$)이 됩니다.
    - 경로의 곡률($C_r$)은 측정 가능한 외란(Feedforward)으로 처리하는 것이 정석입니다.
    - $\dot{\mathbf{x}}_{e} = \mathbf{A}_{e}\mathbf{x}_{e} + \mathbf{B}_{u}\mathbf{u} + \mathbf{B}_{cr}C_r$

$$
\frac{d}{dt} \begin{bmatrix} e_y \\ \dot{e_y} \\ e_\psi \\ \dot{e_\psi} \end{bmatrix} =
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{C_{\alpha f} + C_{\alpha r}}{m v_x} & \frac{C_{\alpha f} + C_{\alpha r}}{m} & \frac{-l_f C_{\alpha f} + l_r C_{\alpha r}}{m v_x} \\
0 & 0 & 0 & 1 \\
0 & \frac{-l_f C_{\alpha f} + l_r C_{\alpha r}}{I_z v_x} & \frac{l_f C_{\alpha f} - l_r C_{\alpha r}}{I_z} & -\frac{l_f^2 C_{\alpha f} + l_r^2 C_{\alpha r}}{I_z v_x}
\end{bmatrix}
\begin{bmatrix} e_y \\ \dot{e_y} \\ e_\psi \\ \dot{e_\psi} \end{bmatrix}
+
\begin{bmatrix} 0 \\ \frac{C_{\alpha f}}{m} \\ 0 \\ \frac{l_f C_{\alpha f}}{I_z} \end{bmatrix}
[\delta_f]
+
\begin{bmatrix} 0 \\ - v_x^2 \\ 0 \\ 0 \end{bmatrix}
[C_r]
$$

구현 메모: `mpc_lateral_controller/lateral_mpc_core.py`에서 곡률 외란은 시간가변 항 `E_d * \kappa_k`로 이산화되어 동역학 제약식에 직접 포함된다.

2. **모델 이산화 (Discretization):**
    - MPC는 이산 시간(Discrete-time) 시스템에서 동작합니다. 따라서 연속 시간(Continuous-time) 상태 공간 모델($\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$)을 MPC의 샘플링 타임($T_s$)에 맞춰 이산 시간 모델($\mathbf{x}_{k+1} = \mathbf{A}_d\mathbf{x}_k + \mathbf{B}_d\mathbf{u}_k$)로 변환해야 합니다. 이는 표준적인 행렬 변환 공식을 사용하면 됩니다.
3. **비용 함수 (Cost Function) 설계:**
    - MPC의 성능을 결정하는 핵심 요소입니다. 일반적으로 다음과 같은 2차 형식(Quadratic form)을 사용합니다.
    $J = \sum_{k=0}^{N-1} (\mathbf{x}_{e,k}^T \mathbf{Q} \mathbf{x}_{e,k} + \mathbf{u}_k^T \mathbf{R} \mathbf{u}_k) + \mathbf{x}_{e,N}^T \mathbf{P} \mathbf{x}_{e,N}$
    - **Q (상태 가중치 행렬):** 상태 오차에 대한 페널티입니다.
        - `Q` 행렬의 `(1,1)` 원소 (ey에 해당) 값을 키우면 횡방향 오차를 줄이는 것을 최우선으로 합니다.
        - `Q` 행렬의 `(3,3)` 원소 (eψ에 해당) 값을 키우면 헤딩 오차를 줄이는 것을 최우선으로 합니다.
    - **R (입력 가중치 행렬):** 제어 입력의 크기에 대한 페널티입니다.
        - `R` 값을 키우면 조향을 부드럽게 하여 승차감을 개선하고 시스템의 안정성을 높입니다. 너무 크면 반응이 느려집니다.
    - **P (종단 상태 가중치):** 예측 구간의 마지막 상태에 대한 페널티입니다. 보통 Q와 비슷하게 설정합니다.
    - **N (예측 호라이즌, Prediction Horizon):** 몇 스텝 앞까지 예측하고 최적화할지 결정합니다. N이 크면 더 먼 미래를 보고 판단하지만 계산량이 증가합니다.
4. **제약 조건 (Constraints) 설정:**
    - MPC의 가장 큰 장점 중 하나입니다.
    - **입력 제약:**
        - `Steering Angle Limit`: `[-40, 40]`도. `u_min ≤ u_k ≤ u_max`
        - `Steering Rate Limit`: 급격한 조향을 막기 위해 1초당 조향각 변화량을 제한할 수 있습니다. `Δu_min ≤ u_k - u_{k-1} ≤ Δu_max`
    - **상태 제약 (선택 사항):**
        - `Lateral Error Limit`: 경로 이탈 방지를 위해 횡방향 오차의 최대값을 제한할 수 있습니다. `ey_min ≤ ey_k ≤ ey_max`

---

### **4단계: 튜닝 및 검증**

1. **가중치 튜닝:** Q와 R 행렬의 값을 조정하며 제어기 성능을 최적화합니다.
    - **팁:** 처음에는 R을 상대적으로 크게 하여 안정적인(보수적인) 제어기에서 시작하고, 점차 Q 값을 높여가며 추종 성능을 개선하는 것이 좋습니다.
2. **성능 비교:** 동일한 경로를 주행하며 기존 Pure Pursuit 제어기와 MPC 제어기의 결과를 비교합니다.
    - **지표:** 횡방향 오차($e_y$), 헤딩 오차($e_\psi$), 조향각($\delta_f$)의 시간 그래프를 그려보세요. MPC가 더 빠르고 안정적으로 오차를 0으로 수렴시키고, 조향각을 더 부드럽게 사용하는지 확인합니다.
3. **언더스티어 개선 확인:** Pure Pursuit가 언더스티어를 보이던 코너 구간에서 MPC가 어떻게 반응하는지 집중적으로 분석합니다. MPC가 경로 곡률을 미리 예측(Feedforward)하고 타이어 동역학을 고려하므로, 코너 진입 시 더 빠르고 정확하게 조향하여 언더스티어를 효과적으로 억제할 수 있을 것입니다.

### **최종 제언**

사용자의 학습 깊이와 최종 목표를 고려했을 때, 다음과 같은 2-Track 전략을 추천합니다.

- **Track A (빠른 구현 및 기본 성능 확보):**
    1. **Kinematic Model**을 채택합니다.
    2. 차량 제원에서 `L` 값을 찾아 모델을 완성합니다.
    3. 이를 기반으로 MPC를 우선 구현하여 전체 제어 루프를 완성하고 동작시키는 것에 집중합니다.
- **Track B (고성능 제어 및 문제 해결):**
    1. **Dynamic Model**을 목표로 설정합니다.
    2. **Step Steer**와 **Steady-State Cornering** 시나리오로 데이터를 수집합니다.
    3. **시스템 식별(System Identification)** 기법으로 $I_z, C_{\alpha f}, C_{\alpha r}$ 파라미터를 추정합니다.
    4. 정확해진 모델을 사용하여 MPC를 설계하고, Pure Pursuit의 언더스티어 문제를 해결하는 고성능 제어기를 완성합니다.

지금까지 정리하신 내용이 매우 훌륭하여 Track B를 수행하실 충분한 역량을 갖추셨다고 생각됩니다. 데이터 수집 및 파라미터 식별 과정이 다소 생소할 수 있지만, 이 단계를 성공적으로 마치면 제어기 성능이 비약적으로 향상되는 것을 경험하실 수 있을 겁니다.