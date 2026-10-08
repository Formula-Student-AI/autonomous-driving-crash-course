#ifndef INCLUDE_VEHICLE_MODELS_FRONT_SLIP_ANGLE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_FRONT_SLIP_ANGLE_MODEL_HPP_

#include <state_lib/state_vector.hpp>
#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/command.hpp"
#include "vehicle_models/types/param.hpp"
#include "vehicle_models/constraint/twist.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector, constraint::Twist Vars>
class FrontSlipAngleModel {
 public:
  /**
   * @brief Initialise front slip angle model
   *
   * @param param Parameters of the motion model
   */
  explicit FrontSlipAngleModel(const Param::Kinematic &kinematic_params)
      : kinematic_params_(kinematic_params) {}

  /**
   * @brief Get front slip angle of vehicle
   *
   * @param state Current vehicle state
   * @return Front slip angle
   */
  double GetSlipAngle(const StateVector &state, const Command &command) const {
    return command.steering_angle -
           std::atan((state(Vars::_v_y) + kinematic_params_.l_F * state(Vars::_v_yaw)) /
                     state(Vars::_v_x));
  }

  /**
   * @brief Get derivative of front slip angle w.r.t. forward and lateral velocity, and angular
   * velocity
   *
   * @param state Current vehicle state
   * @return Derivative of slip angle
   */
  Eigen::Vector3d GetSlipAngleDerivatives(const StateVector &state) const {
    const double helper = state(Vars::_v_y) + kinematic_params_.l_F * state(Vars::_v_yaw);
    const double denominator = state(Vars::_v_x) * state(Vars::_v_x) + helper * helper;

    return {-helper / denominator, state(Vars::_v_x) / denominator,
            kinematic_params_.l_F * state(Vars::_v_x) / denominator};
  }

 protected:
  Param::Kinematic kinematic_params_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_FRONT_SLIP_ANGLE_MODEL_HPP_
