#ifndef INCLUDE_VEHICLE_MODELS_MODELS_ACCELERATION_BASE_ACCELERATION_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_ACCELERATION_BASE_ACCELERATION_HPP_

#include <string>

#include <rclcpp/time.hpp>

#include <state_lib/constraint/vector.hpp>

#include "vehicle_models/downforce_model.hpp"
#include "vehicle_models/drag_model.hpp"
#include "vehicle_models/models/uniform_2d/uniform_2d.hpp"
#include "vehicle_models/models/vehicle_model.hpp"
#include "vehicle_models/powertrain_model.hpp"

#include "vehicle_models/constraint/downforce_model.hpp"
#include "vehicle_models/constraint/drag_model.hpp"
#include "vehicle_models/constraint/powertrain_model.hpp"
#include "vehicle_models/constraint/forward_velocity.hpp"

namespace eufs::vehicle_models {

template <eufs::state::constraint::Vector StateVector,
  constraint::ForwardVelocity Vars,
  constraint::DownforceModel<StateVector> Downforce = DownforceModel<StateVector, Vars>,
  constraint::DragModel<StateVector> Drag = DragModel<StateVector, Vars>,
  constraint::PowertrainModel<StateVector> Powertrain = PowertrainModel<StateVector, Vars>>
class BaseAccelerationModel {
 public:
  /**
   * @brief Initialize the motion model
   *
   * @param param Parameters of the motion model
   * @param process_noise Process noise in the update step
   */
  explicit BaseAccelerationModel(const Param &param)
      : tyre_params_(param.tyre),
        inertia_params_(param.inertia),
        input_ranges_(param.input_ranges),
        downforce_model_(param.inertia, param.aero),
        drag_model_(param.aero),
        powertrain_model_(param.inertia, param.input_ranges, param.powertrain) {}

  /**
   * @brief Propagate vehicle state with time
   *
   * @param state Current vehicle state
   * @param command Latest command
   * @param dt Duration to propagate by
   * @return Updated state vector
   */
  StateVector Update(StateVector &state, const Command &command, const rclcpp::Duration &dt) {
    const Command validated_command = ValidateCommand(command, input_ranges_);

    double total_downforce = downforce_model_.GetTotalDownforce(state);
    double forward_force = powertrain_model_.GetForwardForce(state, validated_command);
    double aero_drag = drag_model_.GetAeroDrag(state);

    const double longitudinal_force =
        forward_force - tyre_params_.rolling_resistance * total_downforce - aero_drag;

    state(Vars::_v_x) += longitudinal_force / inertia_params_.m * dt.seconds();

    BaseUniformMotion2D<StateVector, Vars>::PropagateTwist(state, dt);

    return state;
  }

 protected:
  Param::Tyre tyre_params_;
  Param::Inertia inertia_params_;
  Param::InputRanges input_ranges_;

  Downforce downforce_model_;
  Drag drag_model_;
  Powertrain powertrain_model_;
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_ACCELERATION_BASE_ACCELERATION_HPP_
