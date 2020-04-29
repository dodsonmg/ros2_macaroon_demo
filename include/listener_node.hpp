#ifndef LISTENER_NODE_HPP_
#define LISTENER_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "macaroon_test/visibility.h"

/* macaroons */
#include "macaroons/macaroons.hpp"

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class ListenerNode : public rclcpp::Node
{
public:
  ListenerNode(const std::string & node_name, const std::string & topic_name);
  std::string get_message(void);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string msg_;
  bool fresh_;
};

#endif // LISTENER_NODE_HPP_