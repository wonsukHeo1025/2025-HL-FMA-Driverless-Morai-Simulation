## Lateral MPC Parameter Tuning Guide (Kinematic 우선)

실시간 성능이 충분해졌다면(예: solve time ≈ 1–3 ms), “멀리 보고, 덜 급하게” 제어하도록 파라미터를 조정해 진동을 낮추고 부드럽게 복귀하도록 만듭니다. 이 문서는 Kinematic MPC를 기준으로 설명하며, Dynamic MPC에도 동일 원칙을 적용할 수 있습니다.

---

### 핵심 개념(간단 수식)
- OSQP 목적함수(표준형):
$$
\min_{z} \, \tfrac{1}{2} z^\top P z + q^\top z \quad \text{s.t.} \quad l \le Az \le u
$$
- Kinematic 상태: $x = [e_y,\; e_\psi]^\top$ (횡오차, 헤딩오차)
- 입력: $u = \delta$ (조향각)
- 비용(개념):
$$
J = \sum_{k=0}^{N_p-1} (x_k - x^{ref}_k)^\top Q (x_k - x^{ref}_k) + \sum_{k=0}^{N_c-1} R\,u_k^2 + \sum_{k=0}^{N_c-1} R_\Delta\,\Delta u_k^2 + (x_{N_p}-x^{ref}_{N_p})^\top P (x_{N_p}-x^{ref}_{N_p})
$$
- 직관:
  - $Q, P$↑ → 오차(특히 말단) 줄이려는 힘↑
  - $R$↑ → 조향 세기 억제(작게 조향)
  - $R_\Delta$↑ → 조향 변화율 억제(부드럽게)
  - $N_p$↑ → 멀리 봄(코너 선제대응)
  - $N_c$↓ → 입력 홀드 구간 길어져 스무딩 효과

---

### 파라미터 목록과 효과
- **기본 주기**: $T_s = 1/\text{control\_rate}$ (예: 50 Hz → $T_s=0.02$ s)
- **예측/제어 지평**
  - **$N_p$ (prediction\_horizon)**: 미래를 몇 스텝 볼지. 크게 할수록 먼 미래까지 고려(코너 준비), 계산부하↑
  - **$N_c$ (control\_horizon)**: 몇 스텝만 입력을 자유롭게 둘지. 작게 할수록 이후는 $u_{N_c-1}$ 유지 → 스무딩 효과, 과민반응↓
- **가중치**
  - **$Q$ (Q\_kinematic=[q\_{e\_y}, q\_{e\_\psi}])**: 횡/헤딩 오차 가중. $q_{e_y}$를 과도하게 키우면 “중앙 급복귀” 성향↑
  - **$P$ (P\_kinematic)**: 말단 오차 가중. 안정성과 멀리보기 강화(통상 $P \approx 10Q$)
  - **$R$**: 조향 자체 크기 억제(크면 보수적)
  - **$R_\Delta$**: 조향 변화 억제(크면 부드러움)
- **제약**
  - **delta\_limits=[min, max]**: 조향각 범위(예: ±0.7 rad)
  - **delta\_rate\_max**: 스텝당 변화율 한계(예: 0.01 rad/step). 작을수록 더 스무스
- **기타**
  - **preview\_distance**: 경로 전방 주시 거리(보조 신호). 속도 비례로 약간 키우면 코너 선제대응에 도움

ROS 파라미터 키(kinematic):
- `mpc/prediction_horizon` → $N_p$
- `mpc/control_horizon` → $N_c$
- `mpc/Q_kinematic: [q_ey, q_epsi]` → $Q$
- `mpc/P_kinematic: [p_ey, p_epsi]` → $P$
- `mpc/R`, `mpc/R_delta`
- `mpc/delta_limits: [min, max]`, `mpc/delta_rate_max`
- `mpc/preview_distance`

---

### 증상 → 조치 매핑
- **진동/요요가 보임**(과민 복귀, 직선에서 미세 흔들림)
  - $R_\Delta$↑(예: +10씩), $R$↑(소폭)
  - $N_c$↓(예: 12→10→8)
  - $\text{delta\_rate\_max}$↓(예: 0.01→0.006→0.005)
  - $q_{e_y}$:e\_\psi 비율을 낮춤(예: [100,50]→[70,40])
- **코너 선제대응 부족/늦음**(코너 진입에서 늦게 반응)
  - $N_p$↑(예: 100→125→150 @50Hz)
  - $P$↑(말단 가중 강화), `preview_distance`↑
- **코너 컷팅/라인 이탈 경향**
  - $q_{e_\psi}$↑(헤딩 정렬 가중↑)
  - $R$↑(조향 보수화)
- **반응이 너무 둔함(언더스티어 느낌)**
  - $R_\Delta$↓(예: −10), $\text{delta\_rate\_max}$↑(소폭), $N_c$↑(예: 8→10)

---

### 속도대역별 권장 프리셋(50 Hz 기준)
- **저속 (≤ 8 m/s)**
  - $N_p=100$, $N_c=8$
  - $Q=[60, 35]$, $P=[600, 350]$
  - $R=2.0$, $R_\Delta=30.0$
  - $\text{delta\_limits}=[-0.5, 0.5]$, $\text{delta\_rate\_max}=0.006$
- **중속 (8–20 m/s)**
  - $N_p=125$, $N_c=10$
  - $Q=[70, 40]$, $P=[700, 400]$
  - $R=3.0$, $R_\Delta=40.0$
  - $\text{delta\_limits}=[-0.5, 0.5]$, $\text{delta\_rate\_max}=0.005$
- **고속 (> 20 m/s)**
  - $N_p=150$, $N_c=12$
  - $Q=[80, 45]$, $P=[800, 450]$
  - $R=4.0$, $R_\Delta=50.0$
  - $\text{delta\_limits}=[-0.4, 0.4]$, $\text{delta\_rate\_max}=0.004$

참고: $N_p$는 “미래 시간”으로 생각하세요. 50 Hz에서 $N_p=150 \Rightarrow$ 3 s 가시거리입니다.

---

### 단계별 튜닝 절차(추천)
1) **베이스 설정**부터 시작(중속 프리셋 권장) → 라인 안정화 확인
2) **진동 억제**: $R_\Delta$↑ → 부족 시 $\text{delta\_rate\_max}$↓ → 부족 시 $N_c$↓
3) **코너 준비**: 지연 느낌이면 $N_p$↑, $P$↑, `preview_distance`↑
4) **라인 품질**: 코너 컷팅 시 $q_{e_\psi}$↑, 과도 복귀 시 $q_{e_y}$ 비중↓
5) **미세 확정**: $R$으로 전체 조향 크기 보수화/공격성 조정

각 단계마다 1개 파라미터만 바꾸고, 동일 코스에서 2–3랩 비교로 판단하세요.

---

### YAML 적용 예시(저속 프리셋)
```yaml
mpc:
  prediction_horizon: 100  # Np
  control_horizon: 8       # Nc
  Q_kinematic: [60.0, 35.0]
  P_kinematic: [600.0, 350.0]
  R: 2.0
  R_delta: 30.0
  delta_limits: [-0.5, 0.5]
  delta_rate_max: 0.006
  preview_distance: 5.0
```

런치에서 오버라이드:
```bash
roslaunch mpc_lateral_controller kinematic_mpc_controller.launch \
  _mpc/prediction_horizon:=100 _mpc/control_horizon:=8 \
  _mpc/Q_kinematic:="[60, 35]" _mpc/P_kinematic:="[600, 350]" \
  _mpc/R:=2.0 _mpc/R_delta:=30.0 _mpc/delta_rate_max:=0.006
```

---

### Dynamic MPC에 적용 팁
- 상태가 4개($[e_y, \dot e_y, e_\psi, \dot e_\psi]$)라 $Q$ 설계가 중요합니다.
  - 저속: $q_{e_y}$ 비중↑, 고속: $q_{e_\psi}$와 $\dot e_\psi$ 비중↑
- 나머지 원칙($N_p\uparrow$, $N_c\downarrow$, $R$, $R_\Delta$, 레이트 제한)은 동일합니다.

---

### OSQP 설정 팁(실시간)
- 정밀도: $\text{eps\_abs} = \text{eps\_rel} = 10^{-3}$ → 필요 시 $10^{-4}$
- 반복 수: $\text{max\_iter} = 300\sim800$
- warm start: 항상 사용(이전 해로 초기화)
- polish: 기본 비활성(지연↑), 품질 이슈에 한해 실험적 사용

---

### 체크리스트
- **과민/진동**: $R_\Delta$↑, $\text{delta\_rate\_max}$↓, $N_c$↓
- **코너 대응 늦음**: $N_p$↑, $P$↑, preview↑
- **컷팅**: $q_{e_\psi}$↑, $R$↑
- **둔함**: $R_\Delta$↓, $\text{delta\_rate\_max}$↑(소폭), $N_c$↑

위 가이드로 스무스한 복귀와 코너 선제대응을 동시에 확보할 수 있습니다. 빠른 성능을 바탕으로 $N_p$를 길게 두고, $N_c$는 짧게 유지하는 것이 요점입니다.
