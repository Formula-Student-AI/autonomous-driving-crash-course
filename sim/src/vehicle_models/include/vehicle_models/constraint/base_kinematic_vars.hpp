#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_BASE_KINEMATIC_VARS_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_BASE_KINEMATIC_VARS_HPP_

namespace eufs::vehicle_models::constraint {

template <typename T>
concept BaseKinematicVars = requires() {
  T::_x;
  T::_y;
  T::_yaw;
  T::_v_x;
  T::_v_y;
  T::_a_x;
  T::_a_y;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_BASE_KINEMATIC_VARS_HPP_
