#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_ANGULAR_VELOCITY_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_ANGULAR_VELOCITY_HPP_

namespace eufs::vehicle_models::constraint {

template <typename T>
concept AngularVelocity = requires() {
  T::_v_yaw;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_ANGULAR_VELOCITY_HPP_
