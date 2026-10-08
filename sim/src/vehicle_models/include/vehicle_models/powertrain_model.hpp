#ifndef INCLUDE_VEHICLE_MODELS_POWERTRAIN_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_POWERTRAIN_MODEL_HPP_

#include <algorithm>

#include <state_lib/state_vector.hpp>
#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/command.hpp"
#include "vehicle_models/types/param.hpp"
#include "vehicle_models/constraint/forward_velocity.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector, constraint::ForwardVelocity Vars>
class PowertrainModel {
 public:
  /**
   * @brief Initialise powertrain model
   *
   * @param param Parameters of the motion model
   */
  PowertrainModel(const Param::Inertia &inertia_params, const Param::InputRanges &input_ranges,
                  const Param::Powertrain &powertrain_params)
      : inertia_params_(inertia_params),
        input_ranges_(input_ranges),
        powertrain_params_(powertrain_params) {}

  /**
   * @brief Get driving force of vehicle
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Vehicle's driving force
   */
  double GetForwardForce(const StateVector &state, const Command &command) const {
    return command.acceleration > 0.0
               ? inertia_params_.m * std::min(command.acceleration, GetTractiveForce(state))
               : inertia_params_.m * command.acceleration;
  }

  /**
   * @brief Get the tractive force of the motor
   *
   * @param state Current vehicle state
   * @return Vehicle's tractive force
   */
  double GetTractiveForce(const StateVector &state) const {
    return input_ranges_.acc.max * std::exp(-powertrain_params_.overdrive * state(Vars::_v_x));
  }

  /**
   * @brief Get derivatives of driving force w.r.t. forward velocity
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Derivatives of driving force
   */
  double GetForwardForceDerivative(const StateVector &state, const Command &command) const {
    if (GetTractiveForce(state) > command.acceleration) {
      return -inertia_params_.m * powertrain_params_.overdrive * GetTractiveForce(state);
    }
    return 0.0;
  }

 protected:
  Param::Inertia inertia_params_;
  Param::InputRanges input_ranges_;
  Param::Powertrain powertrain_params_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_POWERTRAIN_MODEL_HPP_
