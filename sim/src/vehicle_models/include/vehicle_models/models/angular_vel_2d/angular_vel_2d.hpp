#ifndef INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_VEL_2D_ANGULAR_VEL_2D_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_VEL_2D_ANGULAR_VEL_2D_HPP_

#include <eigen3/Eigen/Dense>
#include <rclcpp/time.hpp>

#include "vehicle_models/models/angular_vel_2d/base_angular_vel_2d.hpp"
#include "vehicle_models/models/uniform_2d/uniform_2d.hpp"
#include "vehicle_models/models/vehicle_model.hpp"
#include "vehicle_models/process_noise/constant_noise.hpp"

namespace eufs::vehicle_models {

template <typename State, typename Vars,
          template <typename> typename ProcessNoiseGenerator = ConstantNoise>
class AngularVel2D : VehicleModel<State> {
  using StateVector = typename State::Vector;
  using StateMatrix = typename State::Matrix;

 public:
  /**
   * @brief Initialize the motion model
   *
   * @param param Parameters of the motion model
   * @param process_noise Process noise in the update step
   */
  explicit AngularVel2D(const Param &param, const State::EigenMatrix &process_noise)
      : VehicleModel<State>(param, process_noise) {}

  /**
   * @brief Derivatives w.r.t. yaw of the transition from the car's body frame
   *
   * @param state Current vehicle state
   * @param dt Duration used for propagation
   * @return Derivatives of x and y transition
   */
  static Eigen::Vector2d GetFromCarFrameTransitionYawDerivative(const StateVector &state,
                                                                const rclcpp::Duration &dt) {
    const double inv_v_yaw = 1 / state(Vars::_v_yaw);

    const double vel_coeff = state(Vars::_v_x) + state(Vars::_a_x) * dt.seconds();
    const double yaw_coeff = state(Vars::_yaw) + state(Vars::_v_yaw) * dt.seconds();

    const Eigen::Vector2d derivative = {
        vel_coeff * ::cos(yaw_coeff) - state(Vars::_v_x) * ::cos(state(Vars::_yaw)),
        vel_coeff * ::sin(yaw_coeff) - state(Vars::_v_x) * ::sin(state(Vars::_yaw))};

    return inv_v_yaw * derivative;
  }

  /**
   * @brief Derivatives w.r.t. forward velocity of the transition from the car's body frame
   *
   * @param state Current vehicle state
   * @param dt Duration used for propagation
   * @return Derivatives of x and y transition
   */
  static Eigen::Vector2d GetFromCarFrameTransitionVelDerivative(const StateVector &state,
                                                                const rclcpp::Duration &dt) {
    const double inv_v_yaw = 1 / state(Vars::_v_yaw);

    const double yaw_coeff = state(Vars::_yaw) + state(Vars::_v_yaw) * dt.seconds();

    const Eigen::Vector2d derivative = {::sin(yaw_coeff) - ::sin(state(Vars::_yaw)),
                                        -::cos(yaw_coeff) + ::cos(state(Vars::_yaw))};

    return inv_v_yaw * derivative;
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
    const double inv_v_yaw = 1 / state(Vars::_v_yaw);

    const double yaw_coeff = state(Vars::_yaw) + state(Vars::_v_yaw) * dt.seconds();

    const Eigen::Vector2d derivative = {dt.seconds() * ::sin(yaw_coeff),
                                        -dt.seconds() * ::cos(yaw_coeff)};

    return inv_v_yaw * derivative;
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

    const double vel_coeff = state(Vars::_v_x) + state(Vars::_a_x) * dt.seconds();
    const double yaw_coeff = state(Vars::_yaw) + state(Vars::_v_yaw) * dt.seconds();

    const Eigen::Vector2d delta =
        BaseAngularVel2D<StateVector, Vars>::GetFromCarFrameTransition(state, dt);

    const Eigen::Vector2d derivative = {vel_coeff * ::cos(yaw_coeff) * dt.seconds() - delta(0),
                                        vel_coeff * ::sin(yaw_coeff) * dt.seconds() - delta(1)};

    return inv_v_yaw * derivative;
  }
};

};  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_VEL_2D_ANGULAR_VEL_2D_HPP_
