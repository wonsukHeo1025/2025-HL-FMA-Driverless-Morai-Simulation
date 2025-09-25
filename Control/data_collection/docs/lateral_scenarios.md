# Lateral Data Collection Scenarios (Expanded Plan)

This guide specifies expanded, ready-to-run scenario sets for steady-state cornering, step steering, and sine sweep in the refactored data collection system. It targets linear tire region identification (Kv, Caf/Car) and transient dynamics (Iz, damping/natural frequency), with reproducible protocols and analysis cues.

## General execution notes
- Control mode: velocity (recommended). The node will auto build/stabilize speed, then start.
- Use CLI: select 6~9 for lateral scenarios and input parameters when prompted.
- Always log actual steering `steering_angle_deg` and use it as input for estimation. Command `steer_cmd` is auxiliary.
- For steady-state analysis, filter rows with `is_steady_state == True`.
- Safety linearity guardrails: |a_y| < 0.3–0.4 g; reduce amplitudes if saturation or clipping is observed.

## Vehicle speed sets (expanded)
Use four or five speed levels depending on time budget and competition limits (prelim 40 km/h, final 50 km/h).
- Core 4 speeds: 10, 20, 40, 50 km/h
- Expanded 5 speeds: 10, 20, 30, 40, 50 km/h
- Optional offline research (not for competition): 60 km/h

---

## 1) Steady-State Cornering (understeer Kv, Caf/Car)

- Purpose: steady-state gain identification in the linear tire region
- Strategy: alternate ± steering angles, hold after stabilization; use only `is_steady_state == True` windows for regression.

### Parameter template (per speed)
- `target_speed` [km/h]: one of the speed set
- `steering_angles` [deg]: see per-speed lists below (entered as degrees in CLI; code converts to rad)
- `hold_duration` [s]: 12–15 (60 km/h can be 10–12)

### Angle sets by speed
-  5 km/h: ~±26.4 deg (6 12 18 26.4)
- 10 km/h: ~±22.9 deg (5 10 15 22.9)
- 15 km/h: ~±16.6 deg (4 8 12 16.6)
- 20 km/h: ~±13.2 deg (3 6 9 13.2)
- 25 km/h: ~±11.5 deg (3 6 9 11.5)
- 30 km/h: ~±10.3 deg (3 5 8 10.3)
- 35 km/h: ~± 9.2 deg (2 4 6 9.2)
- 40 km/h: ~± 8.6 deg (2 4 6 8.6)
- 45 km/h: ~± 7.7 deg (2 4 6 7.7)
- 50 km/h: ~± 7.2 deg (2 4 6 7.2)

Analysis tips:
- Use `steering_angle_deg` (actual) vs `imu_angular_vel_z` (yaw_rate) and `true_velocity_x` for steady-state mapping.
- Optionally estimate `a_y` from `imu_accel_y` to cross-check linearity and maintain |a_y| < 0.4 g.

---

## 2) Step Steer (transient response, Iz/damping)

- Purpose: characterize yaw-rate step response; extract rise/settle times, overshoot, natural frequency/damping.
- Strategy: two repeats per level, ensure recovery time >> rise time; smaller angles at higher speeds.

### Parameter template (per speed)
- `target_speed` [km/h]
- `step_angles` [deg]
- `step_duration` [s]
- `recovery_duration` [s] (ensure `recovery_duration >= step_rise_time`)
- `step_rise_time` [s]: 0.2 (default)

### Angle/time sets by speed (expanded)
- 10 km/h
  - Angles: ±8, ±12 (repeat each twice)
  - Hold: 6 s; Recovery: 8–10 s
- 20 km/h
  - Angles: ±5, ±10 (repeat each twice)
  - Hold: 6 s; Recovery: 8–10 s
- 30 km/h
  - Angles: ±4, ±8
  - Hold: 6–8 s; Recovery: 10–12 s
- 40 km/h
  - Angles: ±2, ±4, ±6
  - Hold: 6–8 s; Recovery: 10–12 s
- 50 km/h
  - Angles: ±2, ±3.5
  - Hold: 8–10 s; Recovery: 12–15 s
  - Optional offline: 60 km/h → Angles ±1.5, ±3; Hold 8–10 s; Recovery 12–15 s

Recommended sequence (example for 40 km/h): [+2, -2, +4, -4, +6, -6]

Analysis tips:
- Use actual steering as input; fit yaw-rate response to a 2nd-order model to estimate Iz and damping.
- Repeat per angle to average noise; discard runs showing evident saturation/clipping.

---

## 3) Sine Sweep (FRF: gain/phase vs frequency)

- Purpose: obtain lateral dynamics frequency response at fixed speeds.
- Strategy: linear frequency sweep with moderate amplitude in linear region.

### Parameter template (per speed)
- `target_speed` [km/h]
- `freq_start` [Hz], `freq_end` [Hz]
- `sweep_duration` [s]
- `amplitude` [deg] (code expects radians; CLI converts)
- `sweep_type`: linear (recommended)

### Frequency/amplitude sets by speed
- 10 km/h: 0.1 → 1.0 Hz, duration 60–80 s, amplitude 6–8°
- 20 km/h: 0.1 → 1.5 Hz, duration 60–80 s, amplitude 5–6°
- 30–40 km/h: 0.1 → 2.0 Hz, duration 70–90 s, amplitude 4–5°
- 50 km/h: 0.1 → 2.5 Hz, duration 80–100 s, amplitude 3–4°
  - Optional offline: 60 km/h → 0.1 → 3.0 Hz, duration 80–100 s, amplitude 2–3°

Analysis tips:
- Use actual steering as input; compute FRF between steering and yaw-rate (and/or lateral accel).
- Segment the sweep if needed to assume locally stationary frequency; avoid high-frequency tail if SNR degrades.

---

## Example CLI flows

Steady-state (40 km/h):
1) Menu: 6 (Steady-State Cornering)
2) Target speed: 40
3) Angles (deg): 2, -2, 3.5, -3.5, 5, -5, 7, -7
4) Hold duration: 12

Step steer (40 km/h):
1) Menu: 7 (Step Steer)
2) Target speed: 40
3) Step angles (deg): 2, -2, 4, -4, 6, -6
4) Step hold (s): 8
5) Recovery (s): 12

Sine sweep (40 km/h):
1) Menu: 8 (Sine Sweep)
2) Target speed: 40
3) Start freq (Hz): 0.1
4) End freq (Hz): 2.0
5) Sweep duration (s): 80
6) Amplitude (deg): 4

---

## CSV columns of interest
- Use: timestamp, steer_cmd, steering_angle_deg (actual), true_velocity_x, imu_angular_vel_z (yaw_rate), imu_accel_y (lateral accel), is_steady_state
- Keep velocity constant per run; filter by `is_steady_state` for steady-state analysis, and by `scenario_step` windows for step analysis.

## Quality checklist per run
- Speed within ±0.5 km/h during collection
- No actuator clipping; steering within linear-region angles per speed
- High SNR in yaw-rate; avoid excessive IMU saturation
- Repeatability across two runs per condition
