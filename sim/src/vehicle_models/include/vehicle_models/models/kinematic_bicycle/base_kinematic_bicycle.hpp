#ifndef INCLUDE_VEHICLE_MODELS_MODELS_KINEMATIC_BICYCLE_BASE_KINEMATIC_BICYCLE_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_KINEMATIC_BICYCLE_BASE_KINEMATIC_BICYCLE_HPP_

#include <eigen3/Eigen/Dense>
#include <string>
#include <algorithm>
#include "rclcpp/time.hpp"

#include "vehicle_models/types/state.hpp"
#include "vehicle_models/models/vehicle_model.hpp"

namespace eufs::vehicle_models {

template <typename StateVector, typename Vars>
class BaseKinematicBicycle {
 public:
  /**
   * @brief Initialize the motion model
   *
   * @param param Parameters of the motion model
   */
  explicit BaseKinematicBicycle(const Param &param)
      : inertia_params_(param.inertia),
        kinematic_params_(param.kinematic),
        input_ranges_(param.input_ranges),
        aero_params_{param.aero} {}

  /**
   * @brief Propagate vehicle state with time
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  StateVector Update(StateVector &state, const Command &command, const rclcpp::Duration &dt) {
    Command validated_command = ValidateCommand(command, input_ranges_);

    const double velocity =  std::hypot(state(Vars::_v_x), state(Vars::_v_y));
    const double slip_angle = std::atan(
    (kinematic_params_.l_R * std::tan(validated_command.steering_angle)) /
    kinematic_params_.l);

    StateVector state_delta(state);

    state_delta(Vars::_x) = velocity * std::cos(state(Vars::_yaw) + slip_angle);
    state_delta(Vars::_y) = velocity * std::sin(state(Vars::_yaw) + slip_angle);

    state_delta(Vars::_yaw) =
      (velocity * std::tan(validated_command.steering_angle) * std::cos(slip_angle)) /
    kinematic_params_.l;

    // Linear acceleration (not strictly kinematic but needed for some movement to happen)
    state_delta(Vars::_v_x) = PowertrainModel(state, command) / inertia_params_.m;

    // Numerical Integration
    state = state + (state_delta * dt.seconds());

    ValidateState<StateVector, Vars>(state);

    return state;
  }

 protected:
  /**
   * @brief Get longitudinal force
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Longitudinal force
   */
  double PowertrainModel(const StateVector &state, const Command &command) {
    const double acceleration =
        state(Vars::_v_x) <= 0.0 && command.acceleration < 0.0 ? 0.0 : command.acceleration;
    const double longitudinal_force = acceleration * inertia_params_.m - AeroDrag(state);
    return longitudinal_force;
  }

  /**
   * @brief Get aerodynamic drag
   *
   * @param state Current vehicle state
   * @return Aerodynamic drag force
   */
  double AeroDrag(const StateVector &state) const {
    return aero_params_.c_drag * state(Vars::_v_x) * state(Vars::_v_x);
  }

 protected:
  Param::Inertia inertia_params_;
  Param::Kinematic kinematic_params_;
  Param::InputRanges input_ranges_;
  Param::Aero aero_params_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_KINEMATIC_BICYCLE_BASE_KINEMATIC_BICYCLE_HPP_
