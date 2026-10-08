#ifndef INCLUDE_EUFS_LOGGER_ROS_UTILS_HPP_
#define INCLUDE_EUFS_LOGGER_ROS_UTILS_HPP_

// CPP
#include <string>
#include <memory>
#include <utility>

// ROS
#include "rclcpp/parameter.hpp"
#include "rclcpp/node.hpp"

// eufs_logger
#include "eufs_logger/logger.hpp"

namespace eufs::logging {

/**
 * Declares and logs the parameter name
 * @tparam ParamT C++ type of the new parameter
 * @param node A node the parameter is registered for
 * @param name The parameter's name
 * @param default_value The default value of the parameter
 * @param logger A EUFSLoggerPtr object
 * @return The value of the new parameter
 */
template<typename ParamT>
inline auto DeclareAndLogParameter(
    rclcpp::Node *node,
    const std::string &name,
    const ParamT &default_value,
    EUFSLogger &logger) {
  ParamT value = node->declare_parameter(name, default_value);
  logger.info("* Parameter {}: {}", name, default_value);
  return value;
}

/**
 * Creates and registers a new subscription
 * @tparam MsgT The type of incoming messages
 * @tparam CallbackT The type of the callback called for incoming messages
 * @param subscription A reference to a shared pointer to the subscription object
 * @param node A node the new subscription is registered for
 * @param topic_name The subscription's topic name
 * @param callback The callback function for incoming messages
 * @param topic_description The topic's description
 * @param logger A EUFSLoggerPtr object
 * @param options optional subscription options
 * @param qos optional quality-of-service settings
 */
template<typename MsgT, typename CallbackT>
void CreateAndLogSubscription(
    typename rclcpp::Subscription<MsgT>::SharedPtr &subscription,
    rclcpp::Node *node,
    const std::string &topic_name,
    CallbackT callback,
    const std::string &topic_description,
    EUFSLogger &logger,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions(),
    const rclcpp::QoS &qos = rclcpp::QoS(10)) {
  subscription = node->create_subscription<MsgT>(topic_name, qos,
                                                 std::forward<CallbackT>(callback),
                                                 options);
  auto topic_name_remapped =
      node->get_node_topics_interface()->resolve_topic_name(subscription->get_topic_name());
  logger.info("* Subscribing to {}: {}", topic_name_remapped, topic_description);
}

/**
 * Creates and registers a new publisher
 * @tparam MsgT The type of published messages
 * @param publisher A reference to a shared pointer to the publisher object
 * @param node A node the new publisher is registered for
 * @param topic_name The publisher's topic name
 * @param topic_description The publisher's topic description
 * @param qos optional quality-of-service settings
 */
template<typename MsgT>
void CreateAndLogPublisher(typename rclcpp::Publisher<MsgT>::SharedPtr &publisher,
                           rclcpp::Node *node,
                           const std::string &topic_name,
                           const std::string &topic_description,
                           EUFSLogger &logger,
                           const rclcpp::QoS &qos = rclcpp::QoS(10)) {
  publisher = node->create_publisher<MsgT>(topic_name, qos);
  auto topic_name_remapped =
      node->get_node_topics_interface()->resolve_topic_name(publisher->get_topic_name());
  logger.info("* Subscribing to {}: {}", topic_name_remapped, topic_description);
}
}  // namespace eufs::logging

#endif  // INCLUDE_EUFS_LOGGER_ROS_UTILS_HPP_
