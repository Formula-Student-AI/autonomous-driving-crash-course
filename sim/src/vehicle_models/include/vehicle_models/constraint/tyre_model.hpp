#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_TYRE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_TYRE_MODEL_HPP_

#include <concepts>

#include "vehicle_models/types/param.hpp"
#include "vehicle_models/types/command.hpp"

namespace eufs::vehicle_models::constraint {

template <typename T, typename VectorType>
concept TyreModel = eufs::state::constraint::Vector<VectorType> && requires(
    T t,
    const Param::Tyre tyre_params,
    const Param::Inertia inertia_params,
    const Param::Aero aero_params,
    const Param::Kinematic kinematic_params,
    const VectorType vec,
    const Command command) {
  T(tyre_params, inertia_params, aero_params, kinematic_params);
  { t.GetLateralForce(vec, command) } -> std::convertible_to<double>;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_TYRE_MODEL_HPP_
