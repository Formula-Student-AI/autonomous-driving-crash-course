#ifndef INCLUDE_VEHICLE_MODELS_DOWNFORCE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_DOWNFORCE_MODEL_HPP_

#include <state_lib/state_vector.hpp>
#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/param.hpp"
#include "vehicle_models/constraint/forward_velocity.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector, constraint::ForwardVelocity Vars>
class DownforceModel {
 public:
  /**
   * @brief Initialise downforce model
   *
   * @param param Parameters of the motion model
   */
  DownforceModel(const Param::Inertia &inertia_params, const Param::Aero &aero_params)
      : inertia_params_(inertia_params), aero_params_(aero_params) {}

  /**
   * @brief Get net downforce
   *
   * @param state Current vehicle state
   * @return Net downforce
   */
  double GetTotalDownforce(const StateVector &state) const {
    return inertia_params_.g * inertia_params_.m + GetAeroDownforce(state);
  }

  /**
   * @brief Get derivative of total downforce w.r.t. forward velocity
   *
   * @param state Current vehicle state
   * @return Derivative of downforce
   */
  double GetTotalDownforceDerivative(const StateVector &state) const {
    return GetAeroDownforceDerivative(state);
  }

  /**
   * @brief Get downforce due to aerodynamic effects
   *
   * @param state Current vehicle state
   * @return Aerodynamic downforce
   */
  double GetAeroDownforce(const StateVector &state) const {
    return aero_params_.c_down * state(Vars::_v_x) * state(Vars::_v_x);
  }

  /**
   * @brief Get derivative of aerodynamic downforce w.r.t. forward velocity
   *
   * @param state Current vehicle state
   * @return Derivative of aerodynamic downforce
   */
  double GetAeroDownforceDerivative(const StateVector &state) const {
    return 2.0 * aero_params_.c_down * state(Vars::_v_x);
  }

 protected:
  Param::Inertia inertia_params_;
  Param::Aero aero_params_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_DOWNFORCE_MODEL_HPP_
