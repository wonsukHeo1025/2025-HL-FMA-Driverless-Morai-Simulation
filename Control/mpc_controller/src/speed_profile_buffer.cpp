#include "mpc_controller/speed_profile_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mpc_controller {

SpeedProfileBuffer::SpeedProfileBuffer()
{
  resolution_m_ = base_resolution_m_ / static_cast<double>(upsample_factor_);
}

void SpeedProfileBuffer::Configure(double base_resolution_m, std::size_t upsample_factor)
{
  std::lock_guard<std::mutex> lock(mutex_);
  base_resolution_m_ = base_resolution_m;
  upsample_factor_ = std::max<std::size_t>(1, upsample_factor);
  resolution_m_ = base_resolution_m_ / static_cast<double>(upsample_factor_);
  profile_.clear();
}

void SpeedProfileBuffer::Update(const std::vector<double>& values)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (upsample_factor_ > 1) {
    profile_ = Upsample(values);
  } else {
    profile_ = values;
  }
}

void SpeedProfileBuffer::Update(const std_msgs::Float32MultiArray& msg)
{
  std::vector<double> raw;
  raw.reserve(msg.data.size());
  for (const auto value : msg.data) {
    raw.push_back(static_cast<double>(value));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (upsample_factor_ > 1) {
    profile_ = Upsample(raw);
  } else {
    profile_ = std::move(raw);
  }
}

bool SpeedProfileBuffer::Empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return profile_.empty();
}

std::size_t SpeedProfileBuffer::Size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return profile_.size();
}

double SpeedProfileBuffer::Value(std::size_t index) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (profile_.empty()) {
    return 0.0;
  }
  if (index >= profile_.size()) {
    return profile_.back();
  }
  return profile_[index];
}

std::vector<double> SpeedProfileBuffer::BuildTrajectory(std::size_t start_index,
                                                        double current_speed_mps,
                                                        double control_period_s,
                                                        std::size_t horizon_steps) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<double> trajectory(horizon_steps, 0.0);
  if (profile_.empty()) {
    return trajectory;
  }

  const double speed = std::max(0.1, current_speed_mps);

  for (std::size_t k = 0; k < horizon_steps; ++k) {
    const double distance = speed * control_period_s * static_cast<double>(k);
    const double offset = distance / resolution_m_;
    const double exact_index = static_cast<double>(start_index) + offset;
    if (exact_index >= static_cast<double>(profile_.size() - 1)) {
      trajectory[k] = profile_.back();
      continue;
    }

    const std::size_t index_floor = static_cast<std::size_t>(std::floor(exact_index));
    const std::size_t index_ceil = index_floor + 1;
    const double alpha = exact_index - static_cast<double>(index_floor);
    const double v0 = profile_[index_floor];
    const double v1 = profile_[std::min(index_ceil, profile_.size() - 1)];
    trajectory[k] = (1.0 - alpha) * v0 + alpha * v1;
  }

  return trajectory;
}

double SpeedProfileBuffer::DistanceToNextStop(std::size_t start_index,
                                              std::size_t max_lookahead_points,
                                              double stop_threshold) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (profile_.empty() || start_index >= profile_.size()) {
    return std::numeric_limits<double>::infinity();
  }

  std::size_t limit = max_lookahead_points;
  if (upsample_factor_ > 1) {
    if (max_lookahead_points <= std::numeric_limits<std::size_t>::max() / upsample_factor_) {
      limit = max_lookahead_points * upsample_factor_;
    } else {
      limit = std::numeric_limits<std::size_t>::max();
    }
  }
  limit = std::min(profile_.size() - start_index, limit);
  for (std::size_t i = 0; i < limit; ++i) {
    const std::size_t index = start_index + i;
    if (profile_[index] <= stop_threshold) {
      return static_cast<double>(i) * resolution_m_;
    }
  }
  return std::numeric_limits<double>::infinity();
}

double SpeedProfileBuffer::Resolution() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return resolution_m_;
}

std::vector<double> SpeedProfileBuffer::Upsample(const std::vector<double>& raw) const
{
  if (raw.size() < 2 || upsample_factor_ <= 1) {
    return raw;
  }

  const std::size_t factor = upsample_factor_;
  std::vector<double> upsampled((raw.size() - 1) * factor + 1, 0.0);

  for (std::size_t i = 0; i + 1 < raw.size(); ++i) {
    const double v0 = raw[i];
    const double v1 = raw[i + 1];
    for (std::size_t j = 0; j < factor; ++j) {
      const double alpha = static_cast<double>(j) / static_cast<double>(factor);
      const std::size_t index = i * factor + j;
      upsampled[index] = (1.0 - alpha) * v0 + alpha * v1;
    }
  }

  upsampled.back() = raw.back();
  return upsampled;
}

}  // namespace mpc_controller
