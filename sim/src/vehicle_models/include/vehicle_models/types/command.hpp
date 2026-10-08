#ifndef INCLUDE_VEHICLE_MODELS_TYPES_COMMAND_HPP_
#define INCLUDE_VEHICLE_MODELS_TYPES_COMMAND_HPP_

#include <string>

#include "vehicle_models/types/param.hpp"

namespace eufs::vehicle_models {

struct Command {
  inline std::string get_string() const {
    return "(acceleration: " + std::to_string(acceleration) +
           ", steering_angle: " + std::to_string(steering_angle) + ")";
  }

  double acceleration;
  double steering_angle;
};

inline Command ValidateCommand(const Command &command, const Param::InputRanges &input_ranges) {
  Command validated_command = command;
  double max_acc = input_ranges.acc.max;
  double min_acc = input_ranges.acc.min;

  double max_delta = input_ranges.delta.max;
  double min_delta = input_ranges.delta.min;

  validated_command.acceleration = std::fmin(std::fmax(command.acceleration, min_acc), max_acc);
  validated_command.steering_angle =
    std::fmin(std::fmax(command.steering_angle, min_delta), max_delta);

  return validated_command;
}

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_TYPES_COMMAND_HPP_
