#pragma once

#include <cmath>

namespace mpc_controller {

inline double NormalizeAngle(double angle)
{
  const double two_pi = 2.0 * M_PI;
  angle = std::fmod(angle + M_PI, two_pi);
  if (angle < 0.0) {
    angle += two_pi;
  }
  return angle - M_PI;
}

inline double Clamp(double value, double lower, double upper)
{
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

}  // namespace mpc_controller

