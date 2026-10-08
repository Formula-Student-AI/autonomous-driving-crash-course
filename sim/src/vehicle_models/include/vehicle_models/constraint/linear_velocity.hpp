#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_LINEAR_VELOCITY_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_LINEAR_VELOCITY_HPP_

namespace eufs::vehicle_models::constraint {

template <typename T>
concept LinearVelocity = requires() {
  T::_v_x;
  T::_v_y;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_LINEAR_VELOCITY_HPP_
