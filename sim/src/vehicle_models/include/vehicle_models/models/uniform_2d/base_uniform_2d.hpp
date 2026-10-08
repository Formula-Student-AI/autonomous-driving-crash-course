#ifndef INCLUDE_VEHICLE_MODELS_MODELS_UNIFORM_2D_BASE_UNIFORM_2D_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_UNIFORM_2D_BASE_UNIFORM_2D_HPP_

#include <angles/angles.h>

#include <eigen3/Eigen/Dense>
#include <rclcpp/time.hpp>

#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/command.hpp"
#include "vehicle_models/types/param.hpp"

#include "vehicle_models/constraint/pose.hpp"
#include "vehicle_models/constraint/linear_velocity.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector, typename Vars>
  requires constraint::Pose<Vars> && constraint::LinearVelocity<Vars>
class BaseUniformMotion2D {
 public:
  /**
   * @brief Initialize the motion model
   *
   * @param _ Parameters of the motion model
   */
  explicit BaseUniformMotion2D(const Param &) {}

  /**
   * @brief Propagate vehicle state with time
   *
   * @param state Current vehicle state
   * @param _ Latest command
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  static StateVector Propagate(StateVector &state, const Command &, const rclcpp::Duration &dt) {
    PropagateTwist(state, dt);
    PropagateAccel(state, dt);
    return state;
  }

  /**
   * @brief Propagate vehicle state with time
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   */
  StateVector Update(StateVector &state, const Command &command, const rclcpp::Duration &dt) const {
    return BaseUniformMotion2D::Propagate(state, command, dt);
  }

 public:
  /**
   * @brief Propagate twist with time
   *
   * @param state Current vehicle state
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  static StateVector PropagateTwist(StateVector &state, const rclcpp::Duration &dt) {
    state[Vars::_x, Vars::_y] +=
        dt.seconds() * GetFromCarFrameTransition(state, state[Vars::_v_x], state[Vars::_v_y]);

    const double new_angle = state(Vars::_yaw) + dt.seconds() * state(Vars::_v_yaw);
    state(Vars::_yaw) = angles::normalize_angle(new_angle);
    return state;
  }

  /**
   * @brief Propagate acceleration with time
   *
   * @param state Current vehicle state
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  static StateVector PropagateAccel(StateVector &state, const rclcpp::Duration &dt) {
    state[Vars::_v_x, Vars::_v_y] += dt.seconds() * state(Vars::_a_x, Vars::_a_y);

    const double halfTimeSquared = 0.5 * dt.seconds() * dt.seconds();
    state[Vars::_x, Vars::_y] +=
        halfTimeSquared * GetFromCarFrameTransition(state, state[Vars::_a_x], state[Vars::_a_y]);

    return state;
  }

 private:
  /**
   * @brief Propagate vector from car's body frame to a fixed frame
   *
   * @param state Current vehicle state
   * @param x_coeff Magnitude of vector in the car's forward direction
   * @param y_coeff Magnitude of vector in the car's lateral direction
   * @return Displacement in the fixed frame
   */
  static Eigen::Vector2d GetFromCarFrameTransition(const StateVector &state, double x_coeff,
                                                   double y_coeff) {
    const double sy = ::sin(state(Vars::_yaw));
    const double cy = ::cos(state(Vars::_yaw));

    return {x_coeff * cy - y_coeff * sy, x_coeff * sy + y_coeff * cy};
  }
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_UNIFORM_2D_BASE_UNIFORM_2D_HPP_
