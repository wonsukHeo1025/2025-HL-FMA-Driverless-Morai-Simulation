### 목적
- 직선구간 제로크로싱 오실레이션을 **MPC 내부 로직** 중심으로 억제하고, 밴드(두께 있는 경로) 내부에서도 **경로 접선 방향으로 정렬**되도록 함
- 실시간성(50 Hz) 유지: OSQP 직접 백엔드의 빠른 Px(값) 업데이트만 사용, 스파스 구조(P의 nnz)는 불변 유지

---

### 현상과 원인 분석
- 밴드 소프트 제약을 쓰면 밴드 내부(|e_y| ≤ w)에서 입력이 잔잔해져 **lateral error가 유지**됨
- 바깥→안 진입 시점의 **heading(e_psi)** 상태를 유지한 채 이동 → 곧 **반대편 밴드로 재이탈** → 제로크로싱 반복
- 핵심 원인(중요): 차량은 **논홀로노믹(nonholonomic)**. e_y와 e_psi를 독립적으로 제어할 수 없고, 조향은 **헤딩을 통해서만** e_y를 바꾼다
  - 헤딩을 맞추는 동안 충분한 **유지 토크**가 없거나, 입력 스무딩/제약 때문에 헤딩이 맞춰진 직후 **원래 상태로 복귀** → e_y 다시 증가 → 진자처럼 왕복
  - 전형적 징후: inside에서 R, R_delta가 과도하거나, 입력 비용이 0 중심이라 δ_ff 방향 유지력이 약함

---

### 제안 전략: 하이브리드 가중치 스케줄링 + 유지력 강화
비자유도 결합을 고려해, “빨리 맞추고(inside) 잘 유지하며(outside로 나가기 전), 다시 과도하게 흔들지 않게” 만드는 4가지 축을 결합.

#### 1) 밴드 의존 Q 가중 – 연속 블렌딩 + 히스테리시스
- 스위치(inside/outside) 대신 **연속 블렌딩**:
  - s = clamp((|e_y| − w_in) / (w_out − w_in), 0..1)
  - Q(e_y) = (1−s)·Q_inside + s·Q_outside (대각 2×2, SPD)
- **히스테리시스**로 경계 깜빡임 제거: w_in < w_out (예: w_in = 0.9·w, w_out = 1.1·w)
- 의도: inside에선 `Q_y`↓, `Q_psi`↑로 헤딩 정렬 우선. outside에선 `Q_y`,`Q_psi` 모두 크게 하여 빠른 복귀

YAML(추가 제안)
```yaml
mpc:
  band_aware_q_weighting: true
  Q_kinematic_inside_band: [5.0, 800.0]     # [Q_y, Q_psi]
  Q_kinematic_outside_band: [250.0, 400.0]
  band_blend_hysteresis: true
  band_hyst_ratio_inner: 0.9   # w_in = 0.9 * band_base_width
  band_hyst_ratio_outer: 1.1   # w_out = 1.1 * band_base_width
```

구현 포인트(노드/백엔드)
- 노드에서 s 계산 → 활성 Q를 구성 → 백엔드에 전달해 **P의 상태 블록 값만** 업데이트(Px 경로). P의 스파스 구조는 절대 바꾸지 않음

#### 2) 예측지평(Time-varying)에서 헤딩 말기 수렴 강화
- q_scale_front < 1.0, q_scale_back > 1.0, p_scale_end > 1.0
- 의도: 초반은 부드럽게(overshoot 감쇠), 말기는 확실히 경로 접선으로 수렴

YAML(예시)
```yaml
mpc:
  time_varying_weights_enable: true
  q_scale_front: 0.6
  q_scale_back: 1.6
  p_scale_end: 2.0
```

#### 3) 입력 비용을 feedforward 기준으로 중심화: R·(u − δ_ff)^2
- 기존 R·u^2는 직선에서 괜찮지만, 커브/접선 유지에는 **δ_ff 방향** 유지력이 더 중요
- δ_ff = L·κ (필요 시 언더스티어 보상 포함) → R·(u − δ_ff)^2로 바꾸면 헤딩 정렬 후 **유지 토크**가 자연스럽게 생김

구현 포인트(백엔드)
- 목적함수의 U-블록과 q 벡터를 δ_ff 기준으로 재구성 (현재 R_delta(Δu) 항은 동일)

#### 4) 상태 의존 제약/매끄러움 스케줄링
- inside: 빠르게 헤딩 맞춤 → `delta_rate_max`↑, `R_delta`↓
- outside: 과도 없이 복귀 → `delta_rate_max`↓, `R_delta`↑

YAML(예시)
```yaml
mpc:
  state_dep_smoothing_enable: true
  inside_delta_rate_max: 0.025
  inside_R_delta: 15.0
  outside_delta_rate_max: 0.015
  outside_R_delta: 30.0
```

---

### 보강 옵션(중기)
#### A) 로컬 접선 참조 헤딩 주입
- 최근 N m(예: 3~5 m) 중심선 접선의 평균을 참조 헤딩으로 생성 → `x_ref`의 e_psi 목표에 반영
- inside에서도 방향 목표가 존재 → 유지 용이

#### B) Q의 결합 항(오프대각) 소량 추가
- Q = [[Q_y, ρ],[ρ, Q_psi]], |ρ| < sqrt(Q_y·Q_psi) (SPD)
- ρ < 0로 설정 시, 헤딩 오차가 lateral 오차를 줄이는 방향으로 coupling 유도 → overshoot 완화

#### C) preview/curvature 스무딩과 path projection 히스테리시스
- κ, 접선 방향을 저역통과(α=0.7 등) → 목표 방향 흔들림 감소
- 최근접점 인덱스 갱신에 히스테리시스(±M 포인트) 또는 lookahead projection 사용 → 접선 뒤집힘 방지

---

### 구현 가이드(요약)
- OSQP 백엔드
  - P의 스파스 구조 고정. 상태 블록/슬랙 대각 값만 **Px로 갱신**(osqp.update(Px=...))
  - 밴드 의존 Q: 연속 블렌딩 + 히스테리시스 값으로 **활성 Q**를 만들고 상태 블록에 반영
  - 입력 비용 중심화: U-블록/선형항을 `(u − δ_ff)` 기준으로 구성
  - 상태 의존 smoothing: inside/outside에 따라 `delta_rate_max`,`R_delta`를 경계에서만 스위칭(히스테리시스 권장)
- 노드
  - `|e_y|`, `band_base_width`로 s, w_in, w_out 계산
  - 활성 Q, q_scales, p_scale_end 빌드 후 solve 호출 시 전달
  - 로컬 접선 참조 헤딩(optional): path_processor에서 N m 스팬 접선 평균으로 생성 → `x_ref`에 주입

---

### 권장 튜닝(시작점)
- 밴드 연속 블렌딩+히스테리시스
  - w_in = 0.9·w, w_out = 1.1·w
  - Q_inside = [5, 800] → 필요 시 Q_psi 1200까지
  - Q_outside = [250, 400]
- 예측지평 가중
  - q_scale_front = 0.6, q_scale_back = 1.6, p_scale_end = 2.0
- 상태 의존 smoothing
  - inside: delta_rate_max = 0.025, R_delta = 15
  - outside: delta_rate_max = 0.015, R_delta = 30
- 입력 비용 중심화
  - R·(u − δ_ff)^2 (δ_ff = L·κ, 필요 시 Kv v^2 κ 보상)

---

### 검증 체크리스트
- 로그에 활성 Q(inside/outside/blend s) 출력 → 경계 근처 깜빡임 없어야 함
- solver_time, solver_iterations 안정 유지(스파스 구조 고정 확인). P 관련 경고 없어야 함
- 직선 구간: 조향 미세진동 감소, 반대측 재이탈 빈도 저하
- 커브 진입/이탈: 복귀 시간, 최대 |e_y|, rate limit 접촉 빈도 기록

---

### 다음 단계(선택)
- 연속 블렌딩/히스테리시스, 입력 비용 중심화, 상태 의존 smoothing 구현
- 로컬 접선 참조 헤딩 주입
- 필요 시 Q 결합 항, preview/curvature 스무딩, projection 히스테리시스 도입

---

### 요약
- **연속 블렌딩 + 히스테리시스**로 밴드 의존 Q를 안정적으로 적용
- **말기 헤딩 수렴**과 **입력 비용 중심화**로 헤딩을 맞춘 뒤 유지력 확보
- **상태 의존 smoothing**으로 “빨리 맞추고, 과도 없이 유지”
- OSQP의 Px 값 업데이트만 이용해 **실시간성 유지**하면서, PD 외부 보조 없이 **MPC 자체 로직**으로 오실레이션을 줄이는 방향