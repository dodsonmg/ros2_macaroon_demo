#ifndef RESOURCE_BASE_HPP
#define RESOURCE_BASE_HPP

// #include <chrono>
#include <random>

/* macaroons */
#include "macaroons/macaroons.hpp"

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "talker_node.hpp"
#include "listener_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "macaroon_msgs/msg/macaroon.hpp"
#include "macaroon_msgs/msg/macaroons.hpp"
#include "macaroon_msgs/msg/macaroon_resource_request.hpp"
#include "macaroon_msgs/msg/resource_macaroon.hpp"
#include "macaroon_msgs/msg/discharge_macaroon.hpp"
#include "macaroon_msgs/msg/macaroon_command.hpp"

using std::placeholders::_1;

// using namespace macaroons;

// Create a Resource class that subclasses the generic rclcpp::Node base class.
// The Resource class encapsulates one or more Macaroons[, and CHERI token(s)]
class ResourceBase : public rclcpp::Node
{
public:
    ResourceBase(const std::string & node_name, const std::string & authorisation_topic, const std::string & command_topic);

    void publish_authentication_request(const std::string & resource);

    void publish_command(const std::string & command);

    void add_first_party_caveat(const std::string & first_party_caveat = "");
    void add_third_party_caveat(const std::string & location, const std::string & key, const std::string & identifier);

    void initialise_discharge_macaroon(const std::string & location, const std::string & key, const std::string & identifier);

protected:
    void run(void);
    void print_macaroon(void);
    void print_discharge_macaroon(void);
    std::string random_string(std::size_t length);

    // This macaroon is the token held by the node.
    // The node might own the resource, be using the resource, or simply be an intermediary delegating the resource
    macaroons::Macaroon M_;

    // This is a Discharge macaroon that allows the holder to discharge a third party caveat
    // TODO:  Generalise to some kind of dict, so we can use the 'identifier' in the third party caveat
    // to find the correct discharge macaroon.  In this case, we'll assume just one.
    macaroons::Macaroon D_;

    // pub/sub topics
    std::string authentication_topic_;
    std::string command_topic_;

    rclcpp::TimerBase::SharedPtr timer_;

private:

    // create single threaded executor to run both the publisher and subscriber
    // within the same process
    rclcpp::executors::SingleThreadedExecutor exec_;

    // hold the name of the ROS node encapsulated by this class
    // TODO:  It may be appropriate to encapsulate multiple nodes eventually...
    std::string node_name_;

    // used for third party caveats as part of TOFU
    std::string TOFU_key_;
    std::string TOFU_location_;
    std::string TOFU_identifier_;

    // Publishers
    rclcpp::Publisher<macaroon_msgs::msg::MacaroonResourceRequest>::SharedPtr authentication_pub_;
    rclcpp::Publisher<macaroon_msgs::msg::MacaroonCommand>::SharedPtr command_pub_;

    // // Subscribers
    rclcpp::Subscription<macaroon_msgs::msg::ResourceMacaroon>::SharedPtr resource_macaroon_sub_;
    rclcpp::Subscription<macaroon_msgs::msg::DischargeMacaroon>::SharedPtr discharge_macaroon_sub_;

    // callbacks
    void resource_macaroon_cb(const macaroon_msgs::msg::ResourceMacaroon::SharedPtr msg);
    void discharge_macaroon_cb(const macaroon_msgs::msg::DischargeMacaroon::SharedPtr msg);
};

#endif // RESOURCE_BASE_HPP