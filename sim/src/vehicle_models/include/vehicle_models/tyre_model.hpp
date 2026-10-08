#ifndef INCLUDE_VEHICLE_MODELS_TYRE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_TYRE_MODEL_HPP_

#include <state_lib/state_vector.hpp>
#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/types/param.hpp"

#include "vehicle_models/constraint/downforce_component_model.hpp"
#include "vehicle_models/constraint/slip_angle_model.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector, typename Vars,
          constraint::DownforceComponentModel<StateVector> DownforceModel,
          constraint::SlipAngleModel<StateVector> SlipAngleModel>
class TyreModel {
 public:
  /**
   * @brief Initialise tyre model
   *
   * @param param Parameters of the motion model
   */
  TyreModel(const Param::Tyre &tyre_params, const Param::Inertia &inertia_params,
            const Param::Aero &aero_params, const Param::Kinematic &kinematic_params)
      : tyre_params_(tyre_params),
        downforce_model_(kinematic_params, inertia_params, aero_params),
        slip_model_(kinematic_params) {}

  /**
   * @brief Get lateral tyre force
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return lateral tyre force
   */
  double GetLateralForce(const StateVector &state, const Command &command) const {
    const double tyre_downforce = downforce_model_.GetDownforce(state);

    const double mu_y = PacejkaModel(state, command);
    const double lateral_force = tyre_downforce * mu_y;

    return lateral_force;
  }

  /**
   * @brief Get lateral tyre force derivatives w.r.t. forward and lateral velocity, and angular
   * velocity
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return lateral tyre force
   */
  Eigen::Vector3d GetLateralForceDerivatives(const StateVector &state,
                                             const Command &command) const {
    const double tyre_downforce = downforce_model_.GetDownforce(state);
    const double downforce_derivative = downforce_model_.GetDownforceDerivative(state);

    const double mu_y = PacejkaModel(state, command);

    const Eigen::Vector3d helper_derivatives = PacejkaModelDerivatives(state, command);

    return {downforce_derivative * mu_y + tyre_downforce * helper_derivatives(0),
            tyre_downforce * helper_derivatives(1), tyre_downforce * helper_derivatives(2)};
  }

 private:
  /**
   * @brief Get pacejka model values
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Tyre model values
   */
  double PacejkaModel(const StateVector &state, const Command &command) const {
    const double slip_angle = slip_model_.GetSlipAngle(state, command);
    const double mu_y =
        tyre_params_.C * std::sin(tyre_params_.B * std::atan(tyre_params_.A * slip_angle));

    return mu_y;
  }

  /**
   * @brief Get pacejka model derivatives w.r.t. forward and lateral velocity, and angular velocity
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @return Derivatives of tyre model values
   */
  Eigen::Vector3d PacejkaModelDerivatives(const StateVector &state, const Command &command) const {
    const double slip_angle = slip_model_.GetSlipAngle(state, command);
    const Eigen::Vector3d slip_angle_derivatives = slip_model_.GetSlipAngleDerivatives(state);

    return tyre_params_.A * tyre_params_.B * tyre_params_.C *
           cos(tyre_params_.B * std::atan(tyre_params_.A * slip_angle)) *
           (1 / (1 + tyre_params_.A * tyre_params_.A * slip_angle * slip_angle)) *
           slip_angle_derivatives;
  }

 protected:
  Param::Tyre tyre_params_;

  DownforceModel downforce_model_;
  SlipAngleModel slip_model_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_TYRE_MODEL_HPP_
