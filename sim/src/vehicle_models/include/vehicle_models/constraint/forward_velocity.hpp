#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_FORWARD_VELOCITY_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_FORWARD_VELOCITY_HPP_

namespace eufs::vehicle_models::constraint {

template <typename T>
concept ForwardVelocity = requires() {
  T::_v_x;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_FORWARD_VELOCITY_HPP_
