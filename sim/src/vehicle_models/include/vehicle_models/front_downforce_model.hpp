#ifndef INCLUDE_VEHICLE_MODELS_FRONT_DOWNFORCE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_FRONT_DOWNFORCE_MODEL_HPP_

#include <state_lib/state_vector.hpp>
#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/param.hpp"
#include "vehicle_models/downforce_model.hpp"

#include "vehicle_models/constraint/downforce_model.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector,
  typename Vars,
  constraint::DownforceModel<StateVector> Downforce = DownforceModel<StateVector, Vars>>
class FrontDownforceModel {
 public:
  /**
   * @brief Initialise front downforce model
   *
   * @param param Parameters of the motion model
   */
  FrontDownforceModel(const Param::Kinematic &kinematic_params,
                      const Param::Inertia &inertia_params, const Param::Aero &aero_params)
      : kinematic_params_(kinematic_params), downforce_model_(inertia_params, aero_params) {}

  /**
   * @brief Get net downforce at the front of the vehicle
   *
   * @param state Current vehicle state
   * @return Net downforce
   */
  double GetDownforce(const StateVector &state) const {
    const double total_downforce = downforce_model_.GetTotalDownforce(state);

    return kinematic_params_.w_front * total_downforce;
  }

  /**
   * @brief Get derivative of total downforce at the front of the vehicle w.r.t. forward velocity
   *
   * @param state Current vehicle state
   * @return Derivative of downforce
   */
  double GetDownforceDerivative(const StateVector &state) const {
    return kinematic_params_.w_front * downforce_model_.GetTotalDownforceDerivative(state);
  }

 protected:
  Param::Kinematic kinematic_params_;

  Downforce downforce_model_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_FRONT_DOWNFORCE_MODEL_HPP_
