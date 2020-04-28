#include <chrono>

#include "listener_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
ListenerNode::ListenerNode(const std::string & node_name, const std::string & topic_name)
: Node(node_name)
{
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
        [this](const std_msgs::msg::String::SharedPtr msg) -> void
        {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        
        // create a macaroon from the recevied serialised message
        msg_ = msg->data;
        fresh_ = true;
        };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<std_msgs::msg::String>(topic_name, 10, callback);
    msg_ = "";
    fresh_ = false;
}

std::string
ListenerNode::get_message()
{
    if(fresh_)
    {
        fresh_ = false;
        return msg_;
    }
    else
    {
        return "";
    }
}

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(Listener)