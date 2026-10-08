#ifndef INCLUDE_VEHICLE_MODELS_MODELS_UNIFORM_2D_UNIFORM_2D_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_UNIFORM_2D_UNIFORM_2D_HPP_

#include <eigen3/Eigen/Dense>
#include <rclcpp/time.hpp>

#include "vehicle_models/models/uniform_2d/base_uniform_2d.hpp"
#include "vehicle_models/models/vehicle_model.hpp"
#include "vehicle_models/process_noise/constant_noise.hpp"

namespace eufs::vehicle_models {

template <typename State, typename Vars,
          template <typename> typename ProcessNoiseGenerator = ConstantNoise>
class UniformMotion2D : VehicleModel<State> {
  using StateVector = typename State::Vector;
  using StateMatrix = typename State::Matrix;

 public:
  /**
   * @brief Initialize the motion model
   *
   * @param param Parameters of the motion model
   * @param process_noise Process noise in the update step
   */
  explicit UniformMotion2D(const Param &param, const State::EigenMatrix &process_noise)
      : VehicleModel<State, ProcessNoiseGenerator>(param, process_noise), base_model_(param) {}

  /**
   * @brief Propagate vehicle state (including covariance) with time
   *
   * @param state Current vehicle state
   * @param _ Latest command
   * @param dt Duration to propagate by
   */
  State Update(State &state, Command command, const rclcpp::Duration &dt) override {
    StateMatrix jacobian(State::EigenMatrix::Identity());

    base_model_.Update(state.state, command, dt);
    PropagateTwistCovariance(state.state, jacobian, dt);
    PropagateAccelCovariance(state.state, jacobian, dt);

    state.covariance = jacobian() * state.covariance() * jacobian().transpose() +
                       this->process_noise_gen_(state, dt);
    return state;
  }

 public:
  /**
   * @brief Covariance matrix update for propagating twist with time
   *
   * @param state Current vehicle state
   * @param jacobian Jacobian to update according to state transformation
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  static StateVector PropagateTwistCovariance(StateVector &state, StateMatrix &jacobian,
                                              const rclcpp::Duration &dt) {
    jacobian.at(Vars::_x, Vars::_v_x) += ::cos(state(Vars::_yaw)) * dt.seconds();
    jacobian.at(Vars::_x, Vars::_v_y) += -::sin(state(Vars::_yaw)) * dt.seconds();
    jacobian.at(Vars::_y, Vars::_v_x) += ::sin(state(Vars::_yaw)) * dt.seconds();
    jacobian.at(Vars::_y, Vars::_v_y) += ::cos(state(Vars::_yaw)) * dt.seconds();
    jacobian.at(Vars::_yaw, Vars::_v_yaw) += dt.seconds();

    const Eigen::Vector2d derivative =
        GetFromCarFrameYawDerivative(state, state(Vars::_v_x), state(Vars::_v_y));

    jacobian.at(Vars::_x, Vars::_yaw) += dt.seconds() * derivative(0);
    jacobian.at(Vars::_y, Vars::_yaw) += dt.seconds() * derivative(1);

    return state;
  }

  /**
   * @brief Covariance matrix update for propagating acceleration with time
   *
   * @param state Current vehicle state
   * @param jacobian Jacobian to update according to state transformation
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  static StateVector PropagateAccelCovariance(StateVector &state, StateMatrix &jacobian,
                                              const rclcpp::Duration &dt) {
    const double halfTimeSquared = 0.5 * dt.seconds() * dt.seconds();

    jacobian.at(Vars::_v_x, Vars::_a_x) += dt.seconds();
    jacobian.at(Vars::_v_y, Vars::_a_y) += dt.seconds();

    jacobian.at(Vars::_x, Vars::_a_x) += ::cos(state(Vars::_yaw)) * halfTimeSquared;
    jacobian.at(Vars::_x, Vars::_a_y) += -::sin(state(Vars::_yaw)) * halfTimeSquared;
    jacobian.at(Vars::_y, Vars::_a_x) += ::sin(state(Vars::_yaw)) * halfTimeSquared;
    jacobian.at(Vars::_y, Vars::_a_y) += ::cos(state(Vars::_yaw)) * halfTimeSquared;

    const Eigen::Vector2d derivative =
        GetFromCarFrameYawDerivative(state, state(Vars::_a_x), state(Vars::_a_y));

    jacobian.at(Vars::_x, Vars::_yaw) += halfTimeSquared * derivative(0);
    jacobian.at(Vars::_y, Vars::_yaw) += halfTimeSquared * derivative(1);

    return state;
  }

 private:
  /**
   * @brief Derivatives w.r.t. yaw of the transition from the car's body frame
   *
   * @param state Current vehicle state
   * @param dt Duration used for propagation
   * @return Derivatives of x and y transition
   */
  static Eigen::Vector2d GetFromCarFrameYawDerivative(const StateVector &state, double x_coeff,
                                                      double y_coeff) {
    const double sy = ::sin(state(Vars::_yaw));
    const double cy = ::cos(state(Vars::_yaw));

    return {-x_coeff * sy - y_coeff * cy, x_coeff * cy - y_coeff * sy};
  }

  BaseUniformMotion2D<StateVector, Vars> base_model_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_UNIFORM_2D_UNIFORM_2D_HPP_
