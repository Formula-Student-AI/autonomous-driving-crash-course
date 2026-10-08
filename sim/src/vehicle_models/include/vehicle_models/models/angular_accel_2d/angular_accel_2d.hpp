#ifndef INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_ACCEL_2D_ANGULAR_ACCEL_2D_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_ACCEL_2D_ANGULAR_ACCEL_2D_HPP_

#include <eigen3/Eigen/Dense>
#include <rclcpp/time.hpp>

#include "vehicle_models/models/angular_accel_2d/base_angular_accel_2d.hpp"
#include "vehicle_models/models/angular_vel_2d/angular_vel_2d.hpp"
#include "vehicle_models/models/uniform_2d/base_uniform_2d.hpp"
#include "vehicle_models/models/uniform_2d/uniform_2d.hpp"
#include "vehicle_models/models/vehicle_model.hpp"
#include "vehicle_models/process_noise/constant_noise.hpp"

#include "vehicle_models/constraint/pose.hpp"
#include "vehicle_models/constraint/forward_velocity.hpp"
#include "vehicle_models/constraint/angular_velocity.hpp"
#include "vehicle_models/constraint/forward_acceleration.hpp"

namespace eufs::vehicle_models {

template <typename State, typename Vars,
          template <typename> typename ProcessNoiseGenerator = ConstantNoise>
  requires constraint::Pose<Vars>
    && constraint::ForwardVelocity<Vars>
    && constraint::AngularVelocity<Vars>
    && constraint::ForwardAcceleration<Vars>
class AngularAccel2D : VehicleModel<State> {
  using StateVector = typename State::Vector;
  using StateMatrix = typename State::Matrix;

 public:
  /**
   * @brief Initialize the motion model
   *
   * @param param Parameters of the motion model
   * @param process_noise Process noise in the update step
   */
  explicit AngularAccel2D(const Param &param, const State::EigenMatrix &process_noise)
      : VehicleModel<State, ProcessNoiseGenerator>(param, process_noise), base_model_(param) {}

  /**
   * @brief Propagate vehicle state (including covariance) with time
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  State Update(State &state, Command command, const rclcpp::Duration &dt) {
    StateMatrix jacobian(State::EigenMatrix::Identity());

    if (::abs(state.state(Vars::_v_yaw)) > 0.001) {
      Update(state.state, jacobian, command, dt);
    } else {
      BaseUniformMotion2D<StateVector, Vars>::PropagateTwist(state.state, dt);
      BaseUniformMotion2D<StateVector, Vars>::PropagateAccel(state.state, dt);

      UniformMotion2D<State, Vars, ProcessNoiseGenerator>::PropagateTwistCovariance(state.state,
                                                                                    jacobian, dt);
      UniformMotion2D<State, Vars, ProcessNoiseGenerator>::PropagateAccelCovariance(state.state,
                                                                                    jacobian, dt);
    }

    state.covariance = jacobian() * state.covariance() * jacobian().transpose() +
                       this->process_noise_gen_(state, dt);
    return state;
  }

 private:
  /**
   * @brief Helper method to update state and jacobian in place
   *
   * @param state State to update
   * @param jacobian Jacobian to update according to state transformation
   * @param dt Duration used for propagation
   */
  void Update(StateVector &state, StateMatrix &jacobian, const Command &command,
              const rclcpp::Duration &dt) {
    const double delta = dt.seconds();

    state = base_model_.Update(state, command, dt);

    const Eigen::Vector2d yaw_derivative = GetFromCarFrameTransitionYawDerivative(state, dt);
    const Eigen::Vector2d vel_derivative =
        AngularVel2D<State, Vars>::GetFromCarFrameTransitionVelDerivative(state, dt);
    const Eigen::Vector2d accel_derivative = GetFromCarFrameTransitionAccelDerivative(state, dt);
    const Eigen::Vector2d yaw_vel_derivative =
        GetFromCarFrameTransitionAngularYawDerivative(state, dt);

    jacobian.at(Vars::_x, Vars::_yaw) += yaw_derivative(0);
    jacobian.at(Vars::_x, Vars::_v_x) += vel_derivative(0);
    jacobian.at(Vars::_x, Vars::_a_x) += accel_derivative(0);
    jacobian.at(Vars::_x, Vars::_v_yaw) += yaw_vel_derivative(0);

    jacobian.at(Vars::_y, Vars::_yaw) += yaw_derivative(1);
    jacobian.at(Vars::_y, Vars::_v_x) += vel_derivative(1);
    jacobian.at(Vars::_y, Vars::_a_x) += accel_derivative(1);
    jacobian.at(Vars::_y, Vars::_v_yaw) += yaw_vel_derivative(1);

    jacobian.at(Vars::_yaw, Vars::_v_yaw) += delta;
    jacobian.at(Vars::_v_x, Vars::_a_x) += delta;
  }

 private:
  /**
   * @brief Derivatives w.r.t. yaw of the transition from the car's body frame
   *
   * @param state Current vehicle state
   * @param dt Duration used for propagation
   * @return Derivatives of x and y transition
   */
  static Eigen::Vector2d GetFromCarFrameTransitionYawDerivative(const StateVector &state,
                                                                const rclcpp::Duration &dt) {
    const double inv_v_yaw_squared = 1 / (state(Vars::_v_yaw) * state(Vars::_v_yaw));

    const double yaw_coeff = state(Vars::_yaw) + state(Vars::_v_yaw) * dt.seconds();

    const Eigen::Vector2d derivative = {-::sin(yaw_coeff) + ::sin(state(Vars::_yaw)),
                                        ::cos(yaw_coeff) - ::cos(state(Vars::_yaw))};

    const Eigen::Vector2d base_derivative =
        AngularVel2D<State, Vars>::GetFromCarFrameTransitionYawDerivative(state, dt);

    return base_derivative + state(Vars::_a_x) * inv_v_yaw_squared * derivative;
  }

  /**
   * @brief Derivatives w.r.t. forward acceleration of the transition from the car's body frame
   *
   * @param state Current vehicle state
   * @param dt Duration used for propagation
   * @return Derivatives of x and y transition
   */
  static Eigen::Vector2d GetFromCarFrameTransitionAccelDerivative(const StateVector &state,
                                                                  const rclcpp::Duration &dt) {
    const double inv_v_yaw_squared = 1 / (state(Vars::_v_yaw) * state(Vars::_v_yaw));

    const double yaw_coeff = state(Vars::_yaw) + state(Vars::_v_yaw) * dt.seconds();

    const Eigen::Vector2d derivative = {::cos(yaw_coeff) - ::cos(state(Vars::_yaw)),
                                        ::sin(yaw_coeff) - ::sin(state(Vars::_yaw))};

    const Eigen::Vector2d base_delta =
        AngularVel2D<State, Vars>::GetFromCarFrameTransitionAccelDerivative(state, dt);

    return inv_v_yaw_squared * derivative + base_delta;
  }

  /**
   * @brief Derivatives w.r.t. yaw velocity of the transition from the car's body frame
   *
   * @param state Current vehicle state
   * @param dt Duration used for propagation
   * @return Derivatives of x and y transition
   */
  static Eigen::Vector2d GetFromCarFrameTransitionAngularYawDerivative(const StateVector &state,
                                                                       const rclcpp::Duration &dt) {
    const double inv_v_yaw = 1 / state(Vars::_v_yaw);
    const double inv_v_yaw_squared = inv_v_yaw * inv_v_yaw;

    const double yaw_coeff = state(Vars::_yaw) + state(Vars::_v_yaw) * dt.seconds();

    const Eigen::Vector2d derivative = {-::sin(yaw_coeff), ::cos(yaw_coeff)};

    const Eigen::Vector2d delta =
        BaseAngularAccel2D<StateVector, Vars>::GetAccelFromCarFrame(state, dt);

    const Eigen::Vector2d base_delta =
        AngularVel2D<State, Vars>::GetFromCarFrameTransitionAngularYawDerivative(state, dt);

    return state(Vars::_a_x) * inv_v_yaw_squared * derivative * dt.seconds() -
           2.0 * inv_v_yaw * delta + base_delta;
  }

 private:
  BaseAngularAccel2D<StateVector, Vars> base_model_;
};

};  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_ACCEL_2D_ANGULAR_ACCEL_2D_HPP_
