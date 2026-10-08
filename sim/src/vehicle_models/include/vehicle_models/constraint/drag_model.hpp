#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_DRAG_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_DRAG_MODEL_HPP_

#include <concepts>

#include "vehicle_models/types/param.hpp"

namespace eufs::vehicle_models::constraint {

template <typename T, typename VectorType>
concept DragModel = eufs::state::constraint::Vector<VectorType> && requires(
    T t,
    const Param::Aero aero_params,
    const VectorType vec) {
  T(aero_params);
  { t.GetAeroDrag(vec) } -> std::convertible_to<double>;
};  // NOLINT

}  // namespace eufs::vehicle_models::constraint

#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_DRAG_MODEL_HPP_
