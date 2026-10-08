#ifndef INCLUDE_VEHICLE_MODELS_MODELS_DYNAMIC_BICYCLE_BASE_DYNAMIC_BICYCLE_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_DYNAMIC_BICYCLE_BASE_DYNAMIC_BICYCLE_HPP_

#include <eigen3/Eigen/Dense>
#include <string>
#include <algorithm>

#include <rclcpp/time.hpp>

#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/downforce_model.hpp"
#include "vehicle_models/drag_model.hpp"
#include "vehicle_models/front_downforce_model.hpp"
#include "vehicle_models/front_slip_angle_model.hpp"
#include "vehicle_models/models/uniform_2d/uniform_2d.hpp"
#include "vehicle_models/models/vehicle_model.hpp"
#include "vehicle_models/powertrain_model.hpp"
#include "vehicle_models/rear_downforce_model.hpp"
#include "vehicle_models/rear_slip_angle_model.hpp"
#include "vehicle_models/steering_model.hpp"
#include "vehicle_models/types/state.hpp"
#include "vehicle_models/tyre_model.hpp"

#include "vehicle_models/constraint/downforce_model.hpp"
#include "vehicle_models/constraint/drag_model.hpp"
#include "vehicle_models/constraint/powertrain_model.hpp"
#include "vehicle_models/constraint/steering_model.hpp"
#include "vehicle_models/constraint/tyre_model.hpp"
#include "vehicle_models/constraint/twist.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector,
  constraint::Twist Vars,
  constraint::DownforceModel<StateVector> Downforce = DownforceModel<StateVector, Vars>,
  constraint::DragModel<StateVector> Drag = DragModel<StateVector, Vars>,
  constraint::PowertrainModel<StateVector> Powertrain = PowertrainModel<StateVector, Vars>,
  constraint::SteeringModel Steering = SteeringModel,
  constraint::TyreModel<StateVector> FrontTyre =
    TyreModel<StateVector, Vars,
      FrontDownforceModel<StateVector, Vars>, FrontSlipAngleModel<StateVector, Vars>>,
  constraint::TyreModel<StateVector> RearTyre =
    TyreModel<StateVector, Vars,
      RearDownforceModel<StateVector, Vars>, RearSlipAngleModel<StateVector, Vars>>>
class BaseDynamicBicycle {
 public:
  /**
   * @brief Initialize the motion model
   *
   * @param param Parameters of the motion model
   */
  explicit BaseDynamicBicycle(const Param &param)
      : tyre_params_(param.tyre),
        inertia_params_(param.inertia),
        kinematic_params_(param.kinematic),
        input_ranges_(param.input_ranges),
        downforce_model_(param.inertia, param.aero),
        drag_model_(param.aero),
        powertrain_model_(param.inertia, param.input_ranges, param.powertrain),
        steering_model_(param.steering),
        front_tyre_model_(param.tyre, param.inertia, param.aero, param.kinematic),
        rear_tyre_model_(param.tyre, param.inertia, param.aero, param.kinematic) {}

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

    Command smoothed_steering_cmd = validated_command;
    if (!steering_initialized_) {
      prev_steering_angle_ = command.steering_angle;
      steering_initialized_ = true;
    } else {
      smoothed_steering_cmd.steering_angle =
        steering_model_.GetSteeringAngle(prev_steering_angle_, validated_command, dt);
      prev_steering_angle_ = command.steering_angle;
    }

    if (state(Vars::_v_x) < 0.1) {
      state(Vars::_v_yaw) = 0;
      state(Vars::_v_y) = 0;
    }

    StateVector state_copy(state);

    state_copy(Vars::_v_x) +=
      dt.seconds() * GetLongitudinalAcceleration(state, smoothed_steering_cmd);
    state_copy(Vars::_v_y) += dt.seconds() * GetLateralAcceleration(state, smoothed_steering_cmd);
    state_copy(Vars::_v_yaw) += dt.seconds() * GetAngularAcceleration(state, smoothed_steering_cmd);

    state_copy(Vars::_v_x) = std::max(0.0, state_copy(Vars::_v_x));
    state = state_copy;
    BaseUniformMotion2D<StateVector, Vars>::PropagateTwist(state, dt);

    ValidateState<StateVector, Vars>(state);

    return state;
  }

  /**
   * @brief Get longitudinal force
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Longitudinal force
   */
  double GetLongitudinalForce(const StateVector &state, const Command &command) const {
    const double forward_force = powertrain_model_.GetForwardForce(state, command);
    const double total_downforce = downforce_model_.GetTotalDownforce(state);
    const double aero_drag = drag_model_.GetAeroDrag(state);
    if (state(Vars::_v_x) == 0 && state(Vars::_v_y) == 0 && state(Vars::_v_yaw) == 0) {
      return forward_force;
    } else {
        return forward_force - tyre_params_.rolling_resistance * total_downforce - aero_drag;
    }
  }

  /**
   * @brief Get front tyre lateral force (if well-defined)
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Front tyre lateral force, or zero if not well-defined
   */
  double GetFrontTyreLateralForce(const StateVector &state, const Command &command) const {
    return abs(state(Vars::_v_x)) < 0.1 ? 0.0 : front_tyre_model_.GetLateralForce(state, command);
  }

  /**
   * @brief Get rear tyre lateral force (if well-defined)
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Rear tyre lateral force, or zero if not well-defined
   */
  double GetRearTyreLateralForce(const StateVector &state, const Command &command) const {
    return abs(state(Vars::_v_x)) < 0.1 ? 0.0 : rear_tyre_model_.GetLateralForce(state, command);
  }

 protected:
  /**
   * @brief Get longitudinal acceleration
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Longitudinal acceleration
   */
  double GetLongitudinalAcceleration(const StateVector &state, const Command &command) {
    const double longitudinal_force = GetLongitudinalForce(state, command);
    const double front_tyre_lateral_force = GetFrontTyreLateralForce(state, command);

    return (state(Vars::_v_yaw) * state(Vars::_v_y)) +
           (longitudinal_force - std::sin(command.steering_angle) * front_tyre_lateral_force) /
               inertia_params_.m;
  }

  /**
   * @brief Get lateral acceleration
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Lateral acceleration
   */
  double GetLateralAcceleration(const StateVector &state, const Command &command) {
    const double front_tyre_lateral_force = GetFrontTyreLateralForce(state, command);
    const double rear_tyre_lateral_force = GetRearTyreLateralForce(state, command);

    return ((std::cos(command.steering_angle) * front_tyre_lateral_force) +
            rear_tyre_lateral_force) /
               inertia_params_.m -
           (state(Vars::_v_yaw) * state(Vars::_v_x));
  }

  /**
   * @brief Get angular acceleration
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Angular acceleration
   */
  double GetAngularAcceleration(const StateVector &state, const Command &command) {
    const double front_tyre_lateral_force = GetFrontTyreLateralForce(state, command);
    const double rear_tyre_lateral_force = GetRearTyreLateralForce(state, command);

    return (std::cos(command.steering_angle) * front_tyre_lateral_force * kinematic_params_.l_F -
            rear_tyre_lateral_force * kinematic_params_.l_R) /
           inertia_params_.I_z;
  }

 protected:
  bool steering_initialized_ = false;
  double prev_steering_angle_;

  Param::Tyre tyre_params_;
  Param::Inertia inertia_params_;
  Param::Kinematic kinematic_params_;
  Param::InputRanges input_ranges_;

  Downforce downforce_model_;
  Drag drag_model_;
  Powertrain powertrain_model_;
  Steering steering_model_;
  FrontTyre front_tyre_model_;
  RearTyre rear_tyre_model_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_DYNAMIC_BICYCLE_BASE_DYNAMIC_BICYCLE_HPP_
