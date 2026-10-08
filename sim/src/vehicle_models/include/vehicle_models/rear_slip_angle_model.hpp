#ifndef INCLUDE_VEHICLE_MODELS_REAR_SLIP_ANGLE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_REAR_SLIP_ANGLE_MODEL_HPP_

#include <state_lib/state_vector.hpp>
#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/param.hpp"
#include "vehicle_models/constraint/twist.hpp"
#include "vehicle_models/types/command.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector  StateVector, constraint::Twist Vars>
class RearSlipAngleModel {
 public:
  /**
   * @brief Initialise rear slip angle model
   *
   * @param param Parameters of the motion model
   */
  explicit RearSlipAngleModel(const Param::Kinematic &kinematic_params)
      : kinematic_params_(kinematic_params) {}

  /**
   * @brief Get rear slip angle of vehicle
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Rear slip angle
   */
  double GetSlipAngle(const StateVector &state, const Command &) const {
    return -std::atan((state(Vars::_v_y) - kinematic_params_.l_R * state(Vars::_v_yaw)) /
                      state(Vars::_v_x));
  }

  /**
   * @brief Get derivative of rear slip angle w.r.t. forward and lateral velocity, and angular
   * velocity
   *
   * @param state Current vehicle state
   * @return Derivative of slip angle
   */
  Eigen::Vector3d GetSlipAngleDerivatives(const StateVector &state) const {
    const double helper = state(Vars::_v_y) + kinematic_params_.l_R * state(Vars::_v_yaw);
    const double denominator = state(Vars::_v_x) * state(Vars::_v_x) + helper * helper;

    return {-helper / denominator, state(Vars::_v_x) / denominator,
            kinematic_params_.l_R * state(Vars::_v_x) / denominator};
  }

 protected:
  Param::Kinematic kinematic_params_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_REAR_SLIP_ANGLE_MODEL_HPP_
