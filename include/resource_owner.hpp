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
    ResourceOwner(const std::string & node_name, const std::string & authentication_topic, const std::string & command_topic,
                  const std::string location, const std::string identifier);

    void initialise_macaroon(void);
    void initialise_verifier(void);        
    void add_first_party_caveat(const std::string first_party_caveat = "");
    void add_valid_command(const std::string command);
    void add_first_party_caveat_verifier(const std::string first_party_caveat = "");

private:
    void run(void);
    bool verify_macaroon(macaroons::Macaroon resource_macaroon, std::vector<macaroons::Macaroon> discharge_macaroons);
    void publish_resource_and_discharge_macaroons(void);

    macaroons::Verifier V_;

    // Macaroon base properties
    std::string key_;
    std::string identifier_;
    std::string location_;

    // Publishers
    rclcpp::Publisher<macaroon_msgs::msg::ResourceMacaroon>::SharedPtr resource_macaroon_pub_;
    rclcpp::Publisher<macaroon_msgs::msg::DischargeMacaroon>::SharedPtr discharge_macaroon_pub_;

    // Subscribers
    rclcpp::Subscription<macaroon_msgs::msg::MacaroonResourceRequest>::SharedPtr authentication_sub_;
    rclcpp::Subscription<macaroon_msgs::msg::MacaroonCommand>::SharedPtr command_sub_;

    // Callbacks
    void authentication_and_resource_request_cb(const macaroon_msgs::msg::MacaroonResourceRequest::SharedPtr msg);
    void command_cb(const macaroon_msgs::msg::MacaroonCommand::SharedPtr msg);    
};

#endif // RESOURCE_OWNER_HPP