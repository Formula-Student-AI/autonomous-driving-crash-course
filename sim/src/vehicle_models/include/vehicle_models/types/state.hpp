#ifndef INCLUDE_VEHICLE_MODELS_TYPES_STATE_HPP_
#define INCLUDE_VEHICLE_MODELS_TYPES_STATE_HPP_

#include <algorithm>
#include <string>

namespace eufs::vehicle_models {

template <typename StateVector, typename Vars>
inline void ValidateState(StateVector &state) {
  state(Vars::_v_x) = std::max(0.0, state(Vars::_v_x));
}

struct KinematicVariable {
  KinematicVariable operator+(const KinematicVariable &kin_var) const {
    return {x + kin_var.x,       y + kin_var.y,         z + kin_var.z,
            roll + kin_var.roll, pitch + kin_var.pitch, yaw + kin_var.yaw};
  }

  KinematicVariable operator*(const double &constant) const {
    return {constant * x,    constant * y,     constant * z,
            constant * roll, constant * pitch, constant * yaw};
  }

  inline std::string String() const {
    std::string str = "| linear: (x: " + std::to_string(x) + ", y: " + std::to_string(y) +
                      ", z: " + std::to_string(z) + ")\n" +
                      "| angular: (roll: " + std::to_string(roll) +
                      ", pitch: " + std::to_string(pitch) + ", yaw: " + std::to_string(yaw) + ")";
    return str;
  }

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

struct State {
  State operator+(const State &state) const {
    return {position + state.position, velocity + state.velocity,
            acceleration + state.acceleration};
  }

  State operator*(const double &constant) const {
    return {position * constant, velocity * constant, acceleration * constant};
  }

  inline std::string String() const {
    std::string str = "position:\n" + position.String() + "\n" + "velocity:\n" + velocity.String() +
                      "\n" + "acceleration:\n" + acceleration.String();
    return str;
  }

  KinematicVariable position {};
  KinematicVariable velocity {};
  KinematicVariable acceleration {};
};

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_TYPES_STATE_HPP_
