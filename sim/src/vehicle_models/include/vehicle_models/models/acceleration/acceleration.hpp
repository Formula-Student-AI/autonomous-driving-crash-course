#ifndef INCLUDE_VEHICLE_MODELS_MODELS_ACCELERATION_ACCELERATION_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_ACCELERATION_ACCELERATION_HPP_

#include <string>

#include <rclcpp/time.hpp>

#include "vehicle_models/downforce_model.hpp"
#include "vehicle_models/drag_model.hpp"
#include "vehicle_models/models/acceleration/base_acceleration.hpp"
#include "vehicle_models/models/uniform_2d/uniform_2d.hpp"
#include "vehicle_models/models/vehicle_model.hpp"
#include "vehicle_models/powertrain_model.hpp"
#include "vehicle_models/process_noise/constant_noise.hpp"

#include "vehicle_models/constraint/downforce_model.hpp"
#include "vehicle_models/constraint/drag_model.hpp"
#include "vehicle_models/constraint/powertrain_model.hpp"
#include "vehicle_models/constraint/forward_velocity.hpp"

namespace eufs::vehicle_models {

template <typename State,
  constraint::ForwardVelocity Vars,
  constraint::DownforceModel<typename State::Vector> Downforce
    = DownforceModel<typename State::Vector, Vars>,
  constraint::DragModel<typename State::Vector> Drag
    = DragModel<typename State::Vector, Vars>,
  constraint::PowertrainModel<typename State::Vector> Powertrain
    = PowertrainModel<typename State::Vector, Vars>,
  template <typename> typename ProcessNoiseGenerator = ConstantNoise>
class AccelerationModel : public VehicleModel<State> {
  using StateVector = typename State::Vector;
  using StateMatrix = typename State::Matrix;

 public:
  /**
   * @brief Initialize the motion model
   *
   * @param param Parameters of the motion model
   * @param process_noise Process noise in the update step
   */
  explicit AccelerationModel(const Param &param, const StateMatrix &process_noise)
      : VehicleModel<State>(param, process_noise),
        base_model_(param),
        downforce_model_(param.inertia, param.aero),
        drag_model_(param.aero),
        powertrain_model_(param.inertia, param.input_ranges, param.powertrain) {}

  /**
   * @brief Propagate vehicle state (including covariance) with time
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state
   */
  State Update(State &state, Command command, const rclcpp::Duration &dt) {
    base_model_.Update(state.state, command, dt);

    StateMatrix jacobian(State::EigenMatrix::Identity());

    double downforce_derivative = downforce_model_.GetTotalDownforceDerivative(state.state);
    double forward_force_derivative =
        powertrain_model_.GetForwardForceDerivative(state.state, command);
    double aero_drag_derivative = drag_model_.GetAeroDragDerivative(state.state);

    UniformMotion2D<State, Vars>::PropagateTwistCovariance(state.state, jacobian, dt);

    jacobian.at(Vars::_v_x, Vars::_v_x) +=
        (forward_force_derivative - this->param_.tyre.rolling_resistance * downforce_derivative -
         aero_drag_derivative) /
        this->param_.inertia.m * dt.seconds();

    state.covariance = jacobian() * state.covariance() * jacobian().transpose() +
                       this->process_noise_gen_(state, dt);

    return state;
  }

 protected:
  BaseAccelerationModel<StateVector, Vars, Downforce, Drag, Powertrain> base_model_;

  DownforceModel<StateVector, Vars> downforce_model_;
  DragModel<StateVector, Vars> drag_model_;
  PowertrainModel<StateVector, Vars> powertrain_model_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_ACCELERATION_ACCELERATION_HPP_
