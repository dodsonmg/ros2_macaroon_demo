#ifndef RESOURCE_OWNER_HPP
#define RESOURCE_OWNER_HPP

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
class ResourceOwner : public ResourceBase
{
public:
    ResourceOwner(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic);

    void initialise_macaroon(const std::string location, const std::string key, const std::string identifier);
    void initialise_verifier(const std::string key);        
    void add_first_party_caveat(const std::string first_party_caveat = "");
    void add_first_party_caveat_verifier(const std::string first_party_caveat = "");

private:
    void run(void);
    bool verify_macaroon(void);

    MacaroonVerifier V_;
};

#endif // RESOURCE_OWNER_HPP