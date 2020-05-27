#ifndef RESOURCE_USER_HPP
#define RESOURCE_USER_HPP

// #include <chrono>

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "talker_node.hpp"
#include "listener_node.hpp"

/* macaroons */
#include "macaroons/macaroons.hpp"

#include "resource_base.hpp"

// Create a Resource class that subclasses the generic rclcpp::Node base class.
// The Resource class encapsulates a publisher, subscriber, Macaroon[, and CHERI token]
class ResourceUser : public ResourceBase
{
public:
    ResourceUser(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic,
                 const std::string & authentication_topic = "");

    void add_first_party_caveat(const std::string first_party_caveat = "");
    void authentication_and_resource_request(const std::string resource);

private:
    void run(void);

    // used for third party caveats as part of TOFU
    std::string TOFU_key_;
    std::string TOFU_location_;
    std::string TOFU_identifier_;
    std::string TOFU_resource_;

    // Publishers
    rclcpp::Publisher<macaroon_msgs::msg::MacaroonResourceRequest>::SharedPtr authentication_pub_;    
};

#endif // RESOURCE_USER_HPP