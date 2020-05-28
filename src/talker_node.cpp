#include <chrono>

#include "talker_node.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "macaroon_msgs/msg/macaroon.hpp"
// #include "macaroon_msgs/msg/macaroons.hpp"
// #include "macaroon_msgs/msg/macaroon_resource_request.hpp"

using namespace std::chrono_literals;

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
TalkerNode::TalkerNode(const std::string & node_name, const std::string & topic_name)
: Node(node_name)
{
    // Create a publisher to the topic which can be matched with one or more compatible ROS
    // subscribers.
    // Note that not all publishers/subscribers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies (in this case, buffer size of 10?).
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
    macaroons_pub_ = this->create_publisher<macaroon_msgs::msg::Macaroons>(topic_name, 10);
    authentication_pub_ = this->create_publisher<macaroon_msgs::msg::MacaroonResourceRequest>(topic_name, 10);
}

void
TalkerNode::publish_message(const std::string msg_data)
{
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = msg_data;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    pub_->publish(std::move(msg));
}

void
TalkerNode::publish_macaroons_message(const std::vector<std::string> macaroons)
{
    auto data = std::make_unique<macaroon_msgs::msg::Macaroon>();
    auto msg = std::make_unique<macaroon_msgs::msg::Macaroons>();
    for (std::string macaroon : macaroons)
    {
        data->macaroon = macaroon;
        msg->macaroons.push_back(*data);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", macaroon.c_str());
    }

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    macaroons_pub_->publish(std::move(msg));
}

void
TalkerNode::publish_resource_access_request(const std::string key, const std::string location, const std::string identifier)
{
    auto msg = std::make_unique<macaroon_msgs::msg::MacaroonResourceRequest>();
    msg->key = key;
    msg->location = location;
    msg->identifier = identifier;
    RCLCPP_INFO(this->get_logger(), "Publishing resource request (key: %s, location: %s, identifier: %s, resource: %s)", 
        msg->key.c_str(), msg->location.c_str(), msg->identifier.c_str());

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    authentication_pub_->publish(std::move(msg));
}