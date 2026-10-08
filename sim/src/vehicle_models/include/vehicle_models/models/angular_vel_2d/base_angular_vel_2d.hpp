#ifndef INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_VEL_2D_BASE_ANGULAR_VEL_2D_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_VEL_2D_BASE_ANGULAR_VEL_2D_HPP_

#include <eigen3/Eigen/Dense>
#include <rclcpp/time.hpp>

#include "vehicle_models/models/uniform_2d/base_uniform_2d.hpp"
#include "vehicle_models/types/param.hpp"

namespace eufs::vehicle_models {

template <typename StateVector, typename Vars>
class BaseAngularVel2D {
 public:
  /**
   * @brief Initialize the motion model
   *
   * @param _ Parameters of the motion model
   */
  explicit BaseAngularVel2D(const Param &) {}

  /**
   * @brief Propagate vehicle state with time
   *
   * @param state Current vehicle state
   * @param _ Latest command
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  static StateVector Propagate(StateVector &state, const Command &, const rclcpp::Duration &dt) {
    state[Vars::_x, Vars::_y] += GetFromCarFrameTransition(state, dt);

    state(Vars::_yaw) += state(Vars::_v_yaw) * dt.seconds();
    state(Vars::_v_x) += state(Vars::_a_x) * dt.seconds();

    return state;
  }

  /**
   * @brief Propagate vehicle state with time
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  StateVector Update(StateVector &state, const Command &command, const rclcpp::Duration &dt) {
    return BaseAngularVel2D::Propagate(state, command, dt);
  }

  /**
   * @brief Find displacement by converting velocity from the car's body frame
   *
   * @param state Current vehicle state
   * @param dt Duration to propagate by
   * @return Displacement in cartesian plane
   */
  static Eigen::Vector2d GetFromCarFrameTransition(const StateVector &state,
                                                   const rclcpp::Duration &dt) {
    const double inv_v_yaw = 1 / state(Vars::_v_yaw);

    const double vel_coeff = state(Vars::_v_x) + state(Vars::_a_x) * dt.seconds();
    const double yaw_coeff = state(Vars::_yaw) + state(Vars::_v_yaw) * dt.seconds();

    const Eigen::Vector2d delta = {
        vel_coeff * ::sin(yaw_coeff) - state(Vars::_v_x) * ::sin(state(Vars::_yaw)),
        -vel_coeff * ::cos(yaw_coeff) + state(Vars::_v_x) * ::cos(state(Vars::_yaw))};

    return inv_v_yaw * delta;
  }
};

};  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_VEL_2D_BASE_ANGULAR_VEL_2D_HPP_
