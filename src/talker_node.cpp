#include <chrono>

#include "talker_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
TalkerNode::TalkerNode(const std::string & node_name, const std::string & topic_name)
: Node(node_name)
{
    // Create a function for when messages are to be sent.
    auto publish_message =
        [this]() -> void
        {
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg_));
        };

    // Create a publisher to the topic which can be matched with one or more compatible ROS
    // subscribers.
    // Note that not all publishers/subscribers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies (in this case, buffer size of 10?).
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
}

void
TalkerNode::put_message(const std::string & msg)
{
    // initialise msg_ with each call to put_message() due to std::move(msg_) in publish_message()
    msg_ = std::make_unique<std_msgs::msg::String>();
    msg_->data = msg;
}

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(TalkerNode)