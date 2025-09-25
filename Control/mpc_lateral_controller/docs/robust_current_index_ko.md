# 교차로 중첩 구간에서 `current_idx` 강건화 설계

본 문서는 `mpc_lateral_controller`의 `path_processor`가 현재 사용하는 최단거리 기반 인덱스 선택 로직을 분석하고, 교차로처럼 동일 좌표 근방을 경로가 여러 번 지나는 상황(중첩·교차)에서 발생 가능한 점프(jump) 문제를 완화/해결하기 위한 설계를 제안합니다.

## 1) 현행 동작 분석

- 소스: `Control/mpc_lateral_controller/src/mpc_lateral_controller/path_processor.py`
  - `find_closest_point(current_pose)`
    - 차량의 현재 `x, y`와 모든 `path_points[i]` 간 유클리드 거리 `||p_i - p||`를 계산해 최솟값의 인덱스 `i*`를 반환합니다. 즉, 전역 탐색형 최근접 포인트 선택입니다.
  - `calculate_errors(current_pose)`
    - 위에서 얻은 `closest_idx = i*`와 그 다음 점 `i*+1`을 이용해 경로 접선 벡터를 만들고, 
      - 횡오차 `e_y`: 경로 법선 방향으로의 부호 있는 거리
      - 방위오차 `e_psi`: 차량 heading − 경로 heading
      을 계산합니다.
  - 컨트롤 루프(`lateral_mpc_controller_node.py`)에서는
    - 매 주기 `closest_idx = find_closest_point(...)`를 다시 계산하고,
    - 예측 구간 곡률 시퀀스를 `get_curvatures_ahead(closest_idx, ...)`로 얻어 MPC 기준정보로 사용합니다.

요약: 현재 인덱스 선택은 "전 구간 유클리드 최근접점"입니다. 이 방식 자체는 단순·일반 상황에서 안정적으로 동작하지만, 동일 좌표 근방을 경로가 여러 번 지나가는 구간(교차로/헤어핀/랩 코스 일부 중첩)에서는 시간적 연속성이 깨질 수 있습니다.

## 2) 문제 상황 정리

- 사거리(교차로)를 미션상 3번 지나가는 코스라면, 서로 다른 구간의 웨이포인트들이 동일한 XY 근방에 겹칠 수 있습니다(0.5 m 간격 샘플링 가정).
- 차량이 정상적으로 경로를 따라갈 때는 관성과 진행방향 일관성 덕분에 인덱스가 순간 튀어도 지나쳐 무시될 수 있습니다.
- 하지만 차량이 경로를 크게 이탈한 상태로 비정상 각도로 교차로에 진입하면, 단순 최근접 선택이 전혀 다른 "회차의" 웨이포인트로 순간 점프할 위험이 있습니다. 이 경우 잘못된 기준 곡률/속도/오차가 MPC에 입력되어 이상 동작을 유발할 수 있습니다.

## 3) 설계 목표

- 시간적 연속성(temporal continuity)과 진행방향 일관성(directionality)을 보장하며,
- 중첩 구간에서도 "원래 진행 중이던 회차/세그먼트"를 유지/복귀하도록 하고,
- 차량이 경로에서 아주 멀어졌을 땐 안전하게 재초기화(re-init)할 수 있어야 합니다.

## 4) 제안 A: 히스테리시스 + 진행 제약(권장 기본안)

핵심 아이디어: 이전 인덱스(`last_idx`)와 진행 예측치(`idx_hat`)를 이용해, 인덱스 선택을 "가까운 후보들"로 제한하고, 거리 외에 heading 일치도, 예측 인덱스 근접도를 함께 고려하는 비용을 최소화합니다.

- 상태 유지
  - `last_idx`: 직전 주기의 선택 인덱스
  - `ds_mean`: 평균 점간 거리(예: 0.5 m). 실제로는 `path_points`에서 이동 평균으로 갱신 권장
  - `Ts`: 제어 주기
- 진행 예측치
  - `idx_hat = last_idx + round(max(0, v) * Ts / ds_mean)`
- 탐색 윈도우
  - `i ∈ [last_idx - B, last_idx + F]`에서만 후보를 평가
  - 권장 예: `B = 3`, `F = max(5, 1 + ceil(v * preview_distance / ds_mean))`
- 비용함수(다기준)
  - `J(i) = w_d * d_xy(i) + w_psi * |angle_diff(psi_path(i), psi_vehicle)| + w_idx * |i - idx_hat|`
  - 권장 가중치 초기값: `w_d = 1.0`, `w_psi = 0.5~1.0`, `w_idx = 0.2`
- 전방성 게이트(옵션)
  - `(p_i - p_vehicle) · t_i > 0`인 포인트만 후보 허용(경로 접선 `t_i` 기준 전방 포인트 우선)
- 재초기화 안전장치
  - 윈도우 내 최소 `d_xy`가 `d_reinit`(예: 5 m)보다 크면, 일시적으로 전역 최근접 검색으로 재동기화
  - 재초기화 후 `last_idx` 재설정, 이후 다시 윈도우 제약 적용

예시 의사코드:
```python
# inputs: path_points, last_idx, v, Ts, ds_mean, psi_vehicle
# returns: current_idx

def select_index_hysteresis(path_points, last_idx, v, Ts, ds_mean, psi_vehicle,
                             B=3, F_base=5, preview_distance=5.0,
                             w_d=1.0, w_psi=0.8, w_idx=0.2, d_reinit=5.0):
    if not path_points:
        return -1

    idx_hat = last_idx + int(round(max(0.0, v) * Ts / max(1e-3, ds_mean)))
    F = max(F_base, int(np.ceil(max(0.5, v) * preview_distance / max(1e-3, ds_mean))))

    i_min = max(0, last_idx - B)
    i_max = min(len(path_points) - 1, last_idx + F)

    best_i, best_J = None, float('inf')
    for i in range(i_min, i_max + 1):
        pi = path_points[i]
        d = hypot(pi['x'] - x, pi['y'] - y)
        # local path heading from segment (i,i+1) or stored heading
        psi_path = estimate_heading(path_points, i)
        J = w_d * d + w_psi * abs(angle_diff(psi_path, psi_vehicle)) + w_idx * abs(i - idx_hat)

        # optional: forward-only gating
        if i < len(path_points) - 1:
            tx, ty = unit_tangent(path_points[i], path_points[i+1])
            if ((pi['x'] - x) * tx + (pi['y'] - y) * ty) < 0.0:
                J *= 1.5  # or skip

        if J < best_J:
            best_J, best_i = J, i

    # re-init if drifted too far from window
    if best_i is None or distance(path_points[best_i], (x, y)) > d_reinit:
        best_i = argmin_over_all_points_by_distance(path_points, (x, y))

    return best_i
```

장점
- 구현 난이도가 낮고 현재 구조와 호환됩니다.
- 연속성 유지와 잘못된 회차로의 점프를 동시에 억제합니다.

주의
- 역주행/후진 상황을 지원하려면 `max(0, v)` 대신 부호를 허용하고 `B`를 늘리는 등 파라미터 조정이 필요합니다.
- `ds_mean`은 실제 경로 샘플 간격에 맞게 추정·갱신하세요.



## 5) 매우 단순·우아한 대안들(보완용)

- **단조 증가(+ 제한적 후퇴 허용)**: 점프를 구조적으로 막는 1차 안전장치.
  - 규칙: `i_now ∈ [i_last - B_back, i_last + F_fwd]`로 강제(clamp). 일반 주행은 전진 단조, 역주행·정체는 `B_back`만큼만 후퇴 가능.
  - 추천: `B_back=3`, `F_fwd`는 속도 종속으로 5~20 범위.
  - 히스테리시스와 융합: 히스테리시스 비용 최소화 결과를 이 범위로 최종 clamp.


- **거리+헤딩 2항 비용(초경량 비용함수)**: 완전한 히스테리시스 대신도 유효.
  - `J(i) = d_xy(i) + λ · |angle_diff(psi_path(i), psi_vehicle)|`, 후보는 `(idx_hat±W)` 또는 `(i_last±[B,F])`에서만 평가.
  - 추천: `λ = 0.5~1.0`.
  - 히스테리시스와 융합: 히스테리시스 비용의 단순화 버전으로 스위치 가능(리소스 절약 모드).


## 6) 파라미터 가이드(초기값)

- `ds_mean`: 0.5 m (경로 생성 파이프라인과 일치시키기)
- `B`: 3
- `F_base`: 5
- `preview_distance`: 5.0 m
- `w_d`: 1.0, `w_psi`: 0.8, `w_idx`: 0.2
- `d_reinit`: 5.0 m
- 실패 시 재초기화 후, 0.3~0.5 s 동안 전역 최근접을 허용하고, 이후 다시 윈도우 제약 복귀(히스테리시스)

## 7) 구현 포인트(현 코드에 맞춘 삽입 위치)

- 상태 보관: 컨트롤 루프 측(`lateral_mpc_controller_node.py`)에서 `last_idx`를 보관하는 것이 자연스럽습니다. 혹은 `PathProcessor`에 선택 히스토리를 유지하는 멤버를 추가해도 됩니다.
- 대체 엔트리 포인트:
  - `PathProcessor.find_closest_point(...)`를 확장/오버로드하여 `find_index_hysteresis(current_pose, last_idx, v, ...)` 형태로 구현하고, 컨트롤 루프에서 이를 호출.
  - 또는 컨트롤 루프에서 `find_closest_point` 호출 전후로 윈도우·비용 기반 필터링을 적용.
- `psi_path(i)`는 현재도 세그먼트 `(i, i+1)`로부터 산출 가능하므로 추가 저장 없이 계산 가능합니다.

## 8) 다른 교란 요인 체크리스트(터널비전 방지)

- 헤딩 추정/TF
  - MORAI의 heading 필드가 0이므로 `global_yaw_estimator`가 반드시 동작해야 합니다. `reference→base` TF 지연/끊김 시 인덱스 산출이 불안정해질 수 있습니다.
  - 프레임 일관성(`reference`, `base`) 확인. 오차 계산은 `reference` 좌표계 기준입니다.
- 경로 전처리
  - 중복 포인트(0 길이 세그먼트) 제거. `segment_length < 1e-6`은 오류 원인입니다.
  - 곡률 스파이크 방지: 지나치게 촘촘/불균일 샘플링 구간 스무딩.
- 속도/미리보기 설정
  - 매우 작은 `preview_distance`나 너무 큰 `Np`는 잡음/지연에 민감할 수 있음.
- MPC 파라미터/제약
  - 조향 한계(`delta_limits`)와 조향 변화율(`delta_rate_max`)이 너무 타이트하면 일시 인덱스 튐에 복구가 느려질 수 있습니다.
  - 밴드 제약/가중(`band_*`)이 과도하면 e_psi 보정부와 충돌 가능. 필요 시 `band_pd_*` 재조정.
- 재동기화 정책
  - 차량이 경로에서 멀어졌을 때 언제 재초기화할지(`d_reinit`), 재초기화 후 히스테리시스 복귀 타이밍을 명확히 정의.
