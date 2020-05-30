#ifndef RESOURCE_USER_HPP
#define RESOURCE_USER_HPP

#include "resource_base.hpp"

// Create a Resource class that subclasses the generic rclcpp::Node base class.
// The Resource class encapsulates a publisher, subscriber, Macaroon[, and CHERI token]
class ResourceUser : public ResourceBase
{
public:
    ResourceUser(const std::string & authentication_topic, const std::string & command_topic,
                 const std::string & resource);

    void publish_authentication_request(void);
    void publish_command(const std::string & command);

private:
    // Publishers
    rclcpp::Publisher<macaroon_msgs::msg::ResourceRequest>::SharedPtr authentication_pub_;
    rclcpp::Publisher<macaroon_msgs::msg::MacaroonCommand>::SharedPtr command_pub_;
};

#endif // RESOURCE_USER_HPP