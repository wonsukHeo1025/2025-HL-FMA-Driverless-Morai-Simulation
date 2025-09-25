# mpc_controller 테스트 안내

이 디렉터리는 C++/Python 단위 테스트를 모아둡니다.

- `test_speed_profile_buffer.cpp`, `test_path_manager.cpp`, `test_lateral_mpc.cpp`
  - catkin `catkin_make run_tests` 또는 `catkin test`로 실행됩니다.
  - gtest 기반이며 `mpc_controller_core`를 직접 링크하여 핵심 로직을 검증합니다.
- `test_config.py`
  - `pytest`로 실행되며 YAML 파라미터 구조가 의도대로 유지되는지 확인합니다.

## 실행 예시

```bash
catkin_make run_tests_mpc_controller
pytest Control/mpc_controller/tests/test_config.py
```

테스트는 서로 모순되지 않도록 설계되어 있으며, 필요 시 본인이 추가/수정하여 사용하면 됩니다.
