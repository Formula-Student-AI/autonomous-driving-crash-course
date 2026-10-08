#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_SLIP_ANGLE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_SLIP_ANGLE_MODEL_HPP_

#include <concepts>

#include "vehicle_models/types/param.hpp"
#include "vehicle_models/types/command.hpp"

namespace eufs::vehicle_models::constraint {

template <typename T, typename VectorType>
concept SlipAngleModel = eufs::state::constraint::Vector<VectorType> && requires(
    T t,
    const Param::Kinematic &kinematic_params,
    const VectorType vec,
    const Command command) {
  T(kinematic_params);
  { t.GetSlipAngle(vec, command) } -> std::convertible_to<double>;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_SLIP_ANGLE_MODEL_HPP_
