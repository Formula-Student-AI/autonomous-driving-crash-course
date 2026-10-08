#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_POWERTRAIN_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_POWERTRAIN_MODEL_HPP_

#include <concepts>

#include "vehicle_models/types/param.hpp"
#include "vehicle_models/types/command.hpp"

namespace eufs::vehicle_models::constraint {

template <typename T, typename VectorType>
concept PowertrainModel = eufs::state::constraint::Vector<VectorType> && requires(
    T t,
    const Param::Inertia inertia_params,
    const Param::InputRanges input_ranges,
    const Param::Powertrain powertrain_params,
    const VectorType vec,
    const Command command) {
  T(inertia_params, input_ranges, powertrain_params);
  { t.GetForwardForce(vec, command) } -> std::convertible_to<double>;
  { t.GetTractiveForce(vec) } -> std::convertible_to<double>;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_POWERTRAIN_MODEL_HPP_
