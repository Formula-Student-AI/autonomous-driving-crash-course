#ifndef INCLUDE_VEHICLE_MODELS_PROCESS_NOISE_DYNAMIC_NOISE_HPP_
#define INCLUDE_VEHICLE_MODELS_PROCESS_NOISE_DYNAMIC_NOISE_HPP_

#include <eigen3/Eigen/Dense>

#include <utility>

#include <rclcpp/time.hpp>
#include <state_lib/state_mask.hpp>

namespace eufs::vehicle_models {

template <typename State>
class DynamicNoiseGenerator {
 public:
  explicit DynamicNoiseGenerator(State::Matrix process_noise)
      : process_noise_(std::move(process_noise)) {}

  State::EigenMatrix operator()(const State &state, const rclcpp::Duration &dt) {
    constexpr int twist_size = state::GetMaskSize(State::twist_subset);

    Eigen::Matrix<double, twist_size, twist_size> twist_cov;
    twist_cov.setIdentity();

    // Logic taken from the dynamic scaling option in robot_localization
    twist_cov.diagonal() *= state.state(State::twist_subset).norm();

    typename State::Matrix process_noise_copy = process_noise_;
    process_noise_copy[State::twist_subset] =
        twist_cov * process_noise_(State::twist_subset) * twist_cov;

    return dt.seconds() * process_noise_copy;
  }

 protected:
  State::Matrix process_noise_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_PROCESS_NOISE_DYNAMIC_NOISE_HPP_
