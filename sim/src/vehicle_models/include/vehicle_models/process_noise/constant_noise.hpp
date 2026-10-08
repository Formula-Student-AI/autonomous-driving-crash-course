#ifndef INCLUDE_VEHICLE_MODELS_PROCESS_NOISE_CONSTANT_NOISE_HPP_
#define INCLUDE_VEHICLE_MODELS_PROCESS_NOISE_CONSTANT_NOISE_HPP_

#include <utility>

#include <rclcpp/time.hpp>

namespace eufs::vehicle_models {

template <typename State>
class ConstantNoise {
 public:
  explicit ConstantNoise(State::Matrix process_noise) : process_noise_(std::move(process_noise)) {}

  State::EigenMatrix operator()(const State &, const rclcpp::Duration &dt) {
    return dt.seconds() * process_noise_();
  }

 protected:
  State::Matrix process_noise_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_PROCESS_NOISE_CONSTANT_NOISE_HPP_
