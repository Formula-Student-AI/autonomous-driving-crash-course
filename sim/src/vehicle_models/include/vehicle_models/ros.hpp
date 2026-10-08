#ifndef INCLUDE_VEHICLE_MODELS_ROS_HPP_
#define INCLUDE_VEHICLE_MODELS_ROS_HPP_

#include <rclcpp/node.hpp>

#include "vehicle_models/types/param.hpp"

namespace eufs::vehicle_models {

Param SetParamsFromNode(rclcpp::Node &node);

}  // namespace eufs::vehicle_models

#endif  // INCLUDE_VEHICLE_MODELS_ROS_HPP_
