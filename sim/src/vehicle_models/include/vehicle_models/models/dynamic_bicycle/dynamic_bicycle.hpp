#ifndef INCLUDE_VEHICLE_MODELS_MODELS_DYNAMIC_BICYCLE_DYNAMIC_BICYCLE_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_DYNAMIC_BICYCLE_DYNAMIC_BICYCLE_HPP_

#include <eigen3/Eigen/Dense>
#include <string>

#include <rclcpp/time.hpp>

#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/downforce_model.hpp"
#include "vehicle_models/drag_model.hpp"
#include "vehicle_models/front_downforce_model.hpp"
#include "vehicle_models/front_slip_angle_model.hpp"
#include "vehicle_models/models/dynamic_bicycle/base_dynamic_bicycle.hpp"
#include "vehicle_models/models/uniform_2d/uniform_2d.hpp"
#include "vehicle_models/models/vehicle_model.hpp"
#include "vehicle_models/powertrain_model.hpp"
#include "vehicle_models/process_noise/constant_noise.hpp"
#include "vehicle_models/rear_downforce_model.hpp"
#include "vehicle_models/rear_slip_angle_model.hpp"
#include "vehicle_models/tyre_model.hpp"

#include "vehicle_models/constraint/downforce_model.hpp"
#include "vehicle_models/constraint/drag_model.hpp"
#include "vehicle_models/constraint/powertrain_model.hpp"
#include "vehicle_models/constraint/steering_model.hpp"
#include "vehicle_models/constraint/tyre_model.hpp"
#include "vehicle_models/constraint/twist.hpp"

namespace eufs::vehicle_models {

template <typename State,
  constraint::Twist Vars,
  constraint::DownforceModel<typename State::Vector> Downforce
    = DownforceModel<typename State::Vector, Vars>,
  constraint::DragModel<typename State::Vector> Drag
    = DragModel<typename State::Vector, Vars>,
  constraint::PowertrainModel<typename State::Vector> Powertrain
    = PowertrainModel<typename State::Vector, Vars>,
  constraint::SteeringModel Steering = SteeringModel,
  constraint::TyreModel<typename State::Vector> FrontTyre =
    TyreModel<typename State::Vector, Vars,
      FrontDownforceModel<typename State::Vector, Vars>,
        FrontSlipAngleModel<typename State::Vector, Vars>>,
  constraint::TyreModel<typename State::Vector> RearTyre =
    TyreModel<typename State::Vector, Vars,
      RearDownforceModel<typename State::Vector, Vars>,
        RearSlipAngleModel<typename State::Vector, Vars>>,
  template <typename> typename ProcessNoiseGenerator = ConstantNoise>
class DynamicBicycle : public VehicleModel<State> {
  using StateVector = typename State::Vector;
  using StateMatrix = typename State::Matrix;

 public:
  /**
   * @brief Initialize the motion model
   *
   * @param param Parameters of the motion model
   * @param process_noise Process noise in the update step
   */
  explicit DynamicBicycle(const Param &param, const StateMatrix &process_noise)
      : VehicleModel<State>(param, process_noise),
        base_model_(param),
        downforce_model_(param.inertia, param.aero),
        drag_model_(param.aero),
        powertrain_model_(param.inertia, param.input_ranges, param.powertrain),
        front_tyre_model_(param.tyre, param.inertia, param.aero, param.kinematic),
        rear_tyre_model_(param.tyre, param.inertia, param.aero, param.kinematic) {}

  /**
   * @brief Propagate vehicle state (including covariance) with time
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state
   */
  State Update(State &state, Command command, const rclcpp::Duration &dt) {
    // Call first to validate command
    base_model_.Update(state.state, command, dt);

    StateMatrix jacobian(State::EigenMatrix::Identity());

    UpdateLongitudinalVelocity(state.state, jacobian, command, dt);
    UpdateLateralVelocity(state.state, jacobian, command, dt);
    UpdateAngularVelocity(state.state, jacobian, command, dt);

    UniformMotion2D<State, Vars>::PropagateTwistCovariance(state.state, jacobian, dt);

    state.covariance = jacobian() * state.covariance() * jacobian().transpose() +
                       this->process_noise_gen_(state, dt);

    return state;
  }

 private:
  /**
   * @brief Update longitudinal velocity and jacobian corresponding to this motion
   *
   * @param state Current vehicle state
   * @param jacobian Jacobian describing covariance transformation with motion
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state
   */
  StateVector UpdateLongitudinalVelocity(StateVector &state, StateMatrix &jacobian,
                                         const Command &command, const rclcpp::Duration &dt) {
    const double long_force_derivative = GetLongitudinalForceDerivative(state, command);
    const Eigen::Vector3d tyre_model_derivatives =
        GetFrontTyreLateralForceDerivatives(state, command);

    jacobian.at(Vars::_v_x, Vars::_v_x) +=
        dt.seconds() *
        ((long_force_derivative - std::sin(command.steering_angle) * tyre_model_derivatives(0)) /
         this->param_.inertia.m);
    jacobian.at(Vars::_v_x, Vars::_v_y) +=
        dt.seconds() *
        (state(Vars::_v_yaw) - std::sin(command.steering_angle) * tyre_model_derivatives(1));
    jacobian.at(Vars::_v_x, Vars::_v_yaw) +=
        dt.seconds() *
        (state(Vars::_v_y) - std::sin(command.steering_angle) * tyre_model_derivatives(2));

    return state;
  }

  /**
   * @brief Update lateral velocity and jacobian corresponding to this motion
   *
   * @param state Current vehicle state
   * @param jacobian Jacobian describing covariance transformation with motion
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state
   */
  StateVector UpdateLateralVelocity(StateVector &state, StateMatrix &jacobian,
                                    const Command &command, const rclcpp::Duration &dt) {
    const Eigen::Vector3d tyre_front_derivatives =
        GetFrontTyreLateralForceDerivatives(state, command);
    const Eigen::Vector3d tyre_rear_derivatives =
        GetRearTyreLateralForceDerivatives(state, command);

    const Eigen::Vector3d helper =
        (std::cos(command.steering_angle) * tyre_front_derivatives + tyre_rear_derivatives) /
        this->param_.inertia.m;

    jacobian.at(Vars::_v_y, Vars::_v_x) += dt.seconds() * (helper(0) - state(Vars::_v_yaw));
    jacobian.at(Vars::_v_y, Vars::_v_y) += dt.seconds() * helper(1);
    jacobian.at(Vars::_v_y, Vars::_v_yaw) += dt.seconds() * (helper(2) - state(Vars::_v_x));

    return state;
  }

  /**
   * @brief Update angular velocity and jacobian corresponding to this motion
   *
   * @param state Current vehicle state
   * @param jacobian Jacobian describing covariance transformation with motion
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state
   */
  StateVector UpdateAngularVelocity(StateVector &state, StateMatrix &jacobian,
                                    const Command &command, const rclcpp::Duration &dt) {
    const Eigen::Vector3d tyre_front_derivatives =
        GetFrontTyreLateralForceDerivatives(state, command);
    const Eigen::Vector3d tyre_rear_derivatives =
        GetRearTyreLateralForceDerivatives(state, command);

    const Eigen::Vector3d helper =
        (std::cos(command.steering_angle) * tyre_front_derivatives * this->param_.kinematic.l_F -
         tyre_rear_derivatives * this->param_.kinematic.l_R) /
        this->param_.inertia.I_z;

    jacobian.at(Vars::_v_yaw, Vars::_v_x) += dt.seconds() * helper(0);
    jacobian.at(Vars::_v_yaw, Vars::_v_y) += dt.seconds() * helper(1);
    jacobian.at(Vars::_v_yaw, Vars::_v_yaw) += dt.seconds() * helper(2);

    return state;
  }

 protected:
  /**
   * @brief Get derivative of the longitudinal force w.r.t. to forward velocity
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Value of derivative
   */
  double GetLongitudinalForceDerivative(const StateVector &state, const Command &command) const {
    const double forward_force_derivative =
        powertrain_model_.GetForwardForceDerivative(state, command);
    const double total_downforce_derivative = downforce_model_.GetTotalDownforceDerivative(state);
    const double drag_derivative = drag_model_.GetAeroDragDerivative(state);

    return forward_force_derivative -
           this->param_.tyre.rolling_resistance * total_downforce_derivative - drag_derivative;
  }

 public:
  BaseDynamicBicycle<
    StateVector,
    Vars,
    Downforce,
    Drag,
    Powertrain,
    Steering,
    FrontTyre,
    RearTyre> base_model_;

 protected:
  /**
   * @brief Get derivative of the front lateral tyre force w.r.t. to forward and lateral velocity,
   * and angular velocity
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Value of derivative
   */
  Eigen::Vector3d GetFrontTyreLateralForceDerivatives(StateVector &state, const Command &command) {
    return abs(state(Vars::_v_x)) < 0.1
               ? Eigen::Vector3d::Zero()
               : front_tyre_model_.GetLateralForceDerivatives(state, command);
  }

  /**
   * @brief Get derivative of the rear lateral tyre force w.r.t. to forward and lateral velocity,
   * and angular velocity
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Value of derivative
   */
  Eigen::Vector3d GetRearTyreLateralForceDerivatives(StateVector &state, const Command &command) {
    return abs(state(Vars::_v_x)) < 0.1
               ? Eigen::Vector3d::Zero()
               : rear_tyre_model_.GetLateralForceDerivatives(state, command);
  }

  Downforce downforce_model_;
  Drag drag_model_;
  Powertrain powertrain_model_;
  FrontTyre front_tyre_model_;
  RearTyre rear_tyre_model_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_DYNAMIC_BICYCLE_DYNAMIC_BICYCLE_HPP_
