#ifndef INCLUDE_VEHICLE_MODELS_CONSTRAINT_BASE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_CONSTRAINT_BASE_MODEL_HPP_

#include <state_lib/constraint/vector.hpp>

namespace eufs::vehicle_models::constraint {

template<typename T, typename StateVector>
concept BaseModel = state::constraint::Vector<StateVector> && requires(
    T model,
    StateVector state,
    const Command command,
    const rclcpp::Duration dt) {
        T::Propagate(state, command, dt);
        model.Update(state, command, dt);
    };// NOLINT
}  // namespace eufs::vehicle_models::constraint



#endif  // INCLUDE_VEHICLE_MODELS_CONSTRAINT_BASE_MODEL_HPP_
