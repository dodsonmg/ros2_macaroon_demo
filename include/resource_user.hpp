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
                 const std::string & authentication_topic, const std::string & command_topic);

    void add_first_party_caveat(const std::string first_party_caveat = "");

private:
    void run(void);
};

#endif // RESOURCE_USER_HPP