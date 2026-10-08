#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_DOWNFORCE_COMPONENT_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_DOWNFORCE_COMPONENT_MODEL_HPP_

#include <concepts>

#include <state_lib/state_vector.hpp>
#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/param.hpp"

namespace eufs::vehicle_models::constraint {

template <typename T, typename VectorType>
concept DownforceComponentModel = eufs::state::constraint::Vector<VectorType> && requires(
    T t,
    const Param::Kinematic kinematic_params,
    const Param::Inertia inertia_params,
    const Param::Aero aero_params,
    const VectorType vec) {
  T(kinematic_params, inertia_params, aero_params);
  { t.GetDownforce(vec) } -> std::convertible_to<double>;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_DOWNFORCE_COMPONENT_MODEL_HPP_
