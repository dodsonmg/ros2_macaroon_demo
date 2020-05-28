#ifndef TALKER_NODE_HPP_
#define TALKER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "macaroon_msgs/msg/macaroon.hpp"
#include "macaroon_msgs/msg/macaroons.hpp"
#include "macaroon_msgs/msg/macaroon_resource_request.hpp"

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class TalkerNode : public rclcpp::Node
{
public:
  TalkerNode(const std::string & node_name, const std::string & topic_name);
  void publish_message(std::string msg);
  void publish_macaroons_message(const std::vector<std::string> macaroons);
  void publish_resource_access_request(const std::string key, const std::string location, const std::string identifier);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<macaroon_msgs::msg::Macaroons>::SharedPtr macaroons_pub_;
  rclcpp::Publisher<macaroon_msgs::msg::MacaroonResourceRequest>::SharedPtr authentication_pub_;
};

#endif // TALKER_NODE_HPP_