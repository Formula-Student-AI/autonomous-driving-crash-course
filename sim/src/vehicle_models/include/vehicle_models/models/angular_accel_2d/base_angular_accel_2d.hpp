#ifndef INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_ACCEL_2D_BASE_ANGULAR_ACCEL_2D_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_ACCEL_2D_BASE_ANGULAR_ACCEL_2D_HPP_

#include <angles/angles.h>

#include <eigen3/Eigen/Dense>
#include <rclcpp/time.hpp>

#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/models/angular_vel_2d/base_angular_vel_2d.hpp"
#include "vehicle_models/models/uniform_2d/base_uniform_2d.hpp"
#include "vehicle_models/types/command.hpp"
#include "vehicle_models/types/param.hpp"

#include "vehicle_models/constraint/pose.hpp"
#include "vehicle_models/constraint/forward_velocity.hpp"
#include "vehicle_models/constraint/angular_velocity.hpp"
#include "vehicle_models/constraint/forward_acceleration.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector, typename Vars>
  requires constraint::Pose<Vars>
    && constraint::ForwardVelocity<Vars>
    && constraint::AngularVelocity<Vars>
    && constraint::ForwardAcceleration<Vars>
class BaseAngularAccel2D {
 public:
  /**
   * @brief Initialize the motion model
   *
   * @param _ Parameters of the motion model
   */
  explicit BaseAngularAccel2D(const Param &) {}

  /**
   * @brief Propagate vehicle state with time
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  static StateVector Propagate(StateVector &state,
      const Command &command, const rclcpp::Duration &dt) {
    if (::abs(state(Vars::_v_yaw)) > 0.001) {
      Propagate(state, command, dt);
      return state;
    }

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
  StateVector Update(StateVector &state, const Command &command,
                            const rclcpp::Duration &dt) {
    return BaseAngularAccel2D::Propagate(state, command, dt);
  }

 private:
  /**
   * @brief Add the velocity and acceleration propagated from the car's body frame
   *
   * @param state Current vehicle state
   * @param dt Duration to propagate by
   * @return Displacement in cartesian plane
   */
  static Eigen::Vector2d GetFromCarFrameTransition(const StateVector &state,
                                                   const rclcpp::Duration &dt) {
    const Eigen::Vector2d base_delta =
        BaseAngularVel2D<StateVector, Vars>::GetFromCarFrameTransition(state, dt);

    return GetAccelFromCarFrame(state, dt) + base_delta;
  }

 public:
  /**
   * @brief Find displacement by converting acceleration from the car's body frame
   *
   * @param state Current vehicle state
   * @param dt Duration to propagate by
   * @return Displacement in cartesian plane
   */
  static Eigen::Vector2d GetAccelFromCarFrame(const StateVector &state,
                                              const rclcpp::Duration &dt) {
    const double inv_v_yaw_squared = 1 / (state(Vars::_v_yaw) * state(Vars::_v_yaw));

    const double yaw_coeff = state(Vars::_yaw) + state(Vars::_v_yaw) * dt.seconds();

    const Eigen::Vector2d accel_delta = {::cos(yaw_coeff) - ::cos(state(Vars::_yaw)),
                                         ::sin(yaw_coeff) - ::sin(state(Vars::_yaw))};

    return state(Vars::_a_x) * inv_v_yaw_squared * accel_delta;
  }
};

};  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_ANGULAR_ACCEL_2D_BASE_ANGULAR_ACCEL_2D_HPP_
