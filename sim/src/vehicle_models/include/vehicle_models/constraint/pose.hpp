#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_POSE_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_POSE_HPP_

namespace eufs::vehicle_models::constraint {

template <typename T>
concept Pose = requires() {
  T::_x;
  T::_y;
  T::_yaw;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_POSE_HPP_
