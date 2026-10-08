#ifndef INCLUDE_VEHICLE_MODELS_DRAG_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_DRAG_MODEL_HPP_

#include <state_lib/state_vector.hpp>
#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/param.hpp"
#include "vehicle_models/constraint/forward_velocity.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector, constraint::ForwardVelocity Vars>
class DragModel {
 public:
  /**
   * @brief Initialise drag model
   *
   * @param param Parameters of the motion model
   */
  explicit DragModel(const Param::Aero &aero_params) : aero_params_(aero_params) {}

  /**
   * @brief Get aerodynamic drag force
   *
   * @param state Current vehicle state
   * @return Current drag force
   */
  double GetAeroDrag(const StateVector &state) const {
    return aero_params_.c_drag * state(Vars::_v_x) * state(Vars::_v_x);
  }

  /**
   * @brief Get derivative of drag w.r.t. forward velocity
   *
   * @param state Current vehicle state
   * @return Derivative of drag
   */
  double GetAeroDragDerivative(const StateVector &state) const {
    return 2.0 * aero_params_.c_drag * state(Vars::_v_x);
  }

 protected:
  Param::Aero aero_params_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_DRAG_MODEL_HPP_
