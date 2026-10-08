#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_DOWNFORCE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_DOWNFORCE_MODEL_HPP_

#include <concepts>

#include <state_lib/state_vector.hpp>
#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/param.hpp"

namespace eufs::vehicle_models::constraint {

template <typename T, typename VectorType>
concept DownforceModel = eufs::state::constraint::Vector<VectorType> && requires(
    T t,
    const Param::Inertia inertia_params,
    const Param::Aero aero_params,
    const VectorType vec) {
  T(inertia_params, aero_params);
  { t.GetTotalDownforce(vec) } -> std::convertible_to<double>;
  { t.GetAeroDownforce(vec) } -> std::convertible_to<double>;
};  // NOLINT

}  //  namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_DOWNFORCE_MODEL_HPP_
