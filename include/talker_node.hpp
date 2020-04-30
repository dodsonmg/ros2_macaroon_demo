#ifndef TALKER_NODE_HPP_
#define TALKER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "macaroon_test/visibility.h"

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class TalkerNode : public rclcpp::Node
{
public:
  TalkerNode(const std::string & node_name, const std::string & topic_name);
  void publish_message(std::string msg);
  // void put_message(const std::string & msg);

private:
  // size_t count_ = 1;
  // std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  // rclcpp::TimerBase::SharedPtr timer_;
};

#endif // TALKER_NODE_HPP_