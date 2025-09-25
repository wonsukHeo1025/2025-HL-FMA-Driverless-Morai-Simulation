# Offline Analysis Package

CSV 로그 파일로부터 차량 종방향 1차 모델 파라미터를 추정하는 Python 패키지입니다.  
ROS 의존성 없이 독립적으로 동작하며, `CATKIN_IGNORE` 파일로 인해 catkin 빌드 대상에서 제외됩니다.

## 설치

```bash
cd ~/catkin_ws/src/Control/offline_analysis
uv pip install -e .   # or python3 -m pip install -e .
```

## 주요 기능

- **전체 데이터셋 학습**: 모든 CSV 데이터를 사용한 최소자승법 파라미터 추정
- **파라미터 저장/로드**: `.npz` 파일로 파라미터 영구 저장
- **다중 구간 검증**: 시계열 순서를 유지한 연속 구간별 validation
- **시각화**: 실제 vs 예측 속도 그래프 자동 생성

## 사용법

### 1. 파라미터 추정 및 저장

```bash
# 파라미터만 계산하고 저장
python -m offline_analysis.cli --fit-only --save-params params_20250806.npz

# 기본 경로가 아닌 다른 데이터 사용
python -m offline_analysis.cli /path/to/csv/files --fit-only --save-params params.npz --speed-limit 60
```

### 2. 저장된 파라미터로 Validation

```bash
# 단일 구간 검증 (중간 500개 샘플)
python -m offline_analysis.cli --load-params params_20250806.npz --validate-only

# 특정 구간 지정
python -m offline_analysis.cli --load-params params_20250806.npz --validate-only \
    --segment-start 1000 --segment-size 500
```

### 3. 다중 구간 Validation

```bash
# 10개 구간, 각 500샘플씩 검증
python -m offline_analysis.multi_run --load-params params_20250806.npz \
    --segments 10 --segment-size 500

# 파라미터 계산과 동시에 다중 검증
python -m offline_analysis.multi_run --segments 20 --segment-size 1000
```

### 4. 통합 실행 (계산 + 검증)

```bash
# 파라미터 계산 후 바로 검증
python -m offline_analysis.cli /path/to/csv/files --segment-size 500

# 다중 구간으로 한번에
python -m offline_analysis.multi_run --segments 15 --segment-size 750
```

## 파라미터 파일 구조

`.npz` 파일에 저장되는 내용:
- `params`: [A, B, d] 배열 (1차 모델 파라미터)
- `total_samples`: 학습에 사용된 샘플 수
- `speed_limit`: 적용된 속도 제한 (km/h)
- `timestamp`: 생성 시각 (ISO format)

## 옵션 설명

### cli.py 옵션
| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `paths` | data 폴더 | CSV 파일 또는 디렉토리 경로 |
| `--speed-limit` | 50.0 | 속도 제한 필터 (km/h) |
| `--segment-size` | 500 | 검증 구간 크기 (샘플 수) |
| `--segment-start` | 중간 | 검증 시작 인덱스 |
| `--save-params` | - | 파라미터 저장 파일명 |
| `--load-params` | - | 파라미터 로드 파일명 |
| `--fit-only` | False | 파라미터 추정만 수행 |
| `--validate-only` | False | 검증만 수행 (load-params 필요) |

### multi_run.py 옵션
| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--segments` | 10 | 검증 구간 개수 |
| `--segment-size` | 500 | 각 구간 크기 (샘플 수) |
| `--speed-limit` | 60.0 | 속도 제한 필터 (km/h) |
| `--save-params` | - | 파라미터 저장 파일명 |
| `--load-params` | - | 파라미터 로드 파일명 |

## 동작 원리

### 1차 종방향 모델
```
v[k+1] = A * v[k] + B * u[k] + d

여기서:
- v: 속도 (m/s)
- u: 제어 입력 (accel_cmd - brake_cmd)
- A, B, d: 추정할 파라미터
```

### 처리 파이프라인
1. **Load**: 모든 CSV 파일을 DataFrame으로 병합
2. **Preprocess**: 제어 입력 계산, 속도 필터링, NaN 제거
3. **Fit**: 최소자승법으로 파라미터 [A, B, d] 추정
4. **Validate**: 연속 구간에서 시뮬레이션 후 RMSE 계산
5. **Plot**: 실제 vs 예측 그래프 저장

## 출력 파일

- **그래프**: `results/fit_YYYYMMDD_HHMMSS.png`
- **다중 구간 그래프**: `results/multi_segment_YYYYMMDD_HHMMSS.png`
- **파라미터**: 사용자 지정 `.npz` 파일

## 모듈 구조

```
offline_analysis/
├── cli.py          # 메인 CLI 인터페이스
├── multi_run.py    # 다중 구간 검증
├── loader.py       # CSV 파일 로딩
├── preprocess.py   # 데이터 전처리
└── estimator.py    # 파라미터 추정 및 예측
```

## 의존성

- numpy ≥ 1.20
- pandas ≥ 1.3
- matplotlib ≥ 3.5

## 확장 가이드

- **고차 모델**: `preprocess.py`에서 v², |v|v 등 추가 feature 생성
- **다른 추정기**: `estimator.py`에 RLS, Kalman filter 등 구현
- **시나리오별 분석**: CSV의 `source_file` 컬럼으로 시나리오 구분 가능