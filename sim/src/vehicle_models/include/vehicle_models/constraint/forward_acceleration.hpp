#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_FORWARD_ACCELERATION_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_FORWARD_ACCELERATION_HPP_

namespace eufs::vehicle_models::constraint {

template <typename T>
concept ForwardAcceleration = requires() {
  T::_a_x;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_FORWARD_ACCELERATION_HPP_
