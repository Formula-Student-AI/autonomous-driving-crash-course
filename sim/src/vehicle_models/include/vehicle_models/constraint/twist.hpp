#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_TWIST_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_TWIST_HPP_

namespace eufs::vehicle_models::constraint {

template <typename T>
concept Twist = requires() {
  T::_v_x;
  T::_v_y;
  T::_v_yaw;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_TWIST_HPP_
