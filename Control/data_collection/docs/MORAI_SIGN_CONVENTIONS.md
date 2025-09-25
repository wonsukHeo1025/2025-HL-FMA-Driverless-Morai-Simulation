# MORAI Sign Conventions Documentation

## Critical Warning: MORAI Has Inconsistent Sign Conventions!

MORAI simulator uses **different sign conventions** for steering angles across different interfaces. This document clarifies these differences.

## Sign Convention Summary

| Interface | Left Turn (CCW) | Right Turn (CW) | Units |
|-----------|----------------|-----------------|-------|
| **MORAI GUI Display** | Positive (+) | Negative (-) | degrees |
| **Competition_topic.wheel_angle** | **Negative (-)** | **Positive (+)** | degrees |
| **ctrl_cmd.steering** | Positive (+) | Negative (-) | radians |

## Examples

When turning **LEFT** by 10 degrees:
- MORAI GUI shows: **+10°**
- Competition_topic.wheel_angle reports: **-10°** 
- ctrl_cmd.steering expects: **+0.1745 rad**

When turning **RIGHT** by 10 degrees:
- MORAI GUI shows: **-10°**
- Competition_topic.wheel_angle reports: **+10°**
- ctrl_cmd.steering expects: **-0.1745 rad**

## Implementation Notes

### Sending Commands (ctrl_cmd)
```python
# Standard convention: + for left, - for right
steer_rad = np.radians(desired_angle_deg)  # No negation needed
ctrl_cmd.steering = steer_rad
```

### Reading Feedback (Competition_topic)
```python
# Competition_topic uses OPPOSITE convention
# Must negate to match standard convention
actual_steering_deg = -vehicle_state.wheel_angle  # Negate here!
```

### Why This Matters
- Data collection must correctly log steering angles
- Control algorithms need consistent sign conventions
- System identification requires accurate input-output mapping

## Verification
To verify these conventions:
1. Send ctrl_cmd.steering = +0.1745 (10° in radians)
2. Check MORAI GUI: should show ~+10°
3. Check Competition_topic.wheel_angle: will show ~-10°

## Date Discovered
2025-01-08 - Found during lateral system identification development