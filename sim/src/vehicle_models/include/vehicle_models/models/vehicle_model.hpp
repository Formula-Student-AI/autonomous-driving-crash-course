#ifndef INCLUDE_VEHICLE_MODELS_MODELS_VEHICLE_MODEL_HPP_
#define INCLUDE_VEHICLE_MODELS_MODELS_VEHICLE_MODEL_HPP_

#include <memory>
#include <string>

#include <rclcpp/time.hpp>

#include "vehicle_models/process_noise/constant_noise.hpp"
#include "vehicle_models/types/command.hpp"
#include "vehicle_models/types/param.hpp"

namespace eufs::vehicle_models {

template <typename State, template <typename> typename ProcessNoiseGenerator = ConstantNoise>
class VehicleModel {
 public:
  explicit VehicleModel(const Param &param, State::Matrix process_noise)
      : param_(param), process_noise_gen_(process_noise) {}

  virtual State Update(State &state, Command command, const rclcpp::Duration &dt) = 0;

  Param &GetParam() { return param_; }

 protected:
  Param param_;
  ProcessNoiseGenerator<State> process_noise_gen_;
};
template <typename State>
using VehicleModelPtr = std::unique_ptr<VehicleModel<State>>;
}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_MODELS_VEHICLE_MODEL_HPP_
