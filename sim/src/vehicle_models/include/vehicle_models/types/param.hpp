#ifndef INCLUDE_VEHICLE_MODELS_TYPES_PARAM_HPP_
#define INCLUDE_VEHICLE_MODELS_TYPES_PARAM_HPP_

#include <string>

#include "yaml-cpp/yaml.h"

namespace eufs::vehicle_models {

struct Param {
  void SetFromYaml(const std::string &yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);

    inertia = config["inertia"].as<Param::Inertia>();
    kinematic = config["kinematic"].as<Param::Kinematic>();
    tyre = config["tyre"].as<Param::Tyre>();
    powertrain = config["powertrain"].as<Param::Powertrain>();
    steering = config["steering"].as<Param::Steering>();
    aero = config["aero"].as<Param::Aero>();
    input_ranges = config["input_ranges"].as<Param::InputRanges>();
  }

  struct Inertia {
    double m;
    double g;
    double I_z;
  };

  struct Kinematic {
    double l;
    double w_front;
    double l_F;
    double l_R;
    double axle_width;

    Kinematic() = default;

    Kinematic(const double l, const double w_front, const double axle_width)
        : l(l),
          w_front(w_front),
          l_F(w_front * l),
          l_R((1 - w_front) * l),
          axle_width(axle_width) {}
  };

  struct Tyre {
    double A;
    double B;
    double C;
    double radius;
    double rolling_resistance;
  };

  struct Powertrain {
    double overdrive;
  };

  struct Steering {
    double max_rate;
    double max_angle;
  };

  struct Aero {
    double c_down;
    double c_drag;
  };

  struct InputRanges {
    struct Range {
      double min;
      double max;
    };

    Range acc;
    Range vel;
    Range delta;
  };

  Inertia inertia;
  Kinematic kinematic;
  Tyre tyre;
  Powertrain powertrain;
  Steering steering;
  Aero aero;
  InputRanges input_ranges;
};

}  // namespace eufs::vehicle_models

namespace YAML {
template <>
struct convert<eufs::vehicle_models::Param::Inertia> {
  static bool decode(const Node &node, eufs::vehicle_models::Param::Inertia &cType) {
    cType.m = node["m"].as<double>();
    cType.g = node["g"].as<double>();
    cType.I_z = node["I_z"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::vehicle_models::Param::Kinematic> {
  static bool decode(const Node &node, eufs::vehicle_models::Param::Kinematic &cType) {
    cType = eufs::vehicle_models::Param::Kinematic(
      node["l"].as<double>(),
      node["w_front"].as<double>(),
      node["axle_width"].as<double>());
    return true;
  }
};

template <>
struct convert<eufs::vehicle_models::Param::Tyre> {
  static bool decode(const Node &node, eufs::vehicle_models::Param::Tyre &cType) {
    cType.A = node["A"].as<double>();
    cType.B = node["B"].as<double>();
    cType.C = node["C"].as<double>();
    cType.radius = node["radius"].as<double>();
    cType.rolling_resistance = node["rolling_resistance"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::vehicle_models::Param::Powertrain> {
  static bool decode(const Node &node, eufs::vehicle_models::Param::Powertrain &cType) {
    cType.overdrive = node["overdrive"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::vehicle_models::Param::Steering> {
  static bool decode(const Node &node, eufs::vehicle_models::Param::Steering &cType) {
    cType.max_rate = node["max_rate"].as<double>();
    cType.max_angle = node["max_angle"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::vehicle_models::Param::Aero> {
  static bool decode(const Node &node, eufs::vehicle_models::Param::Aero &cType) {
    cType.c_down = node["C_down"].as<double>();
    cType.c_drag = node["C_drag"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::vehicle_models::Param::InputRanges> {
  static bool decode(const Node &node, eufs::vehicle_models::Param::InputRanges &cType) {
    cType.acc.min = node["acceleration"]["min"].as<double>();
    cType.acc.max = node["acceleration"]["max"].as<double>();
    cType.vel.min = node["velocity"]["min"].as<double>();
    cType.vel.max = node["velocity"]["max"].as<double>();
    cType.delta.min = node["steering"]["min"].as<double>();
    cType.delta.max = node["steering"]["max"].as<double>();
    return true;
  }
};

}  // namespace YAML

#endif  // INCLUDE_VEHICLE_MODELS_TYPES_PARAM_HPP_
