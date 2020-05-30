#ifndef RESOURCE_BASE_HPP
#define RESOURCE_BASE_HPP

// #include <chrono>
#include <random>

/* macaroons */
#include "macaroons/macaroons.hpp"

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "macaroon_msgs/msg/macaroon.hpp"
#include "macaroon_msgs/msg/resource_macaroon.hpp"
#include "macaroon_msgs/msg/discharge_macaroon.hpp"
#include "macaroon_msgs/msg/macaroon_command.hpp"
#include "macaroon_msgs/msg/resource_request.hpp"

using std::placeholders::_1;

// Create a Resource class that subclasses the generic rclcpp::Node base class.
// The Resource class encapsulates one or more Macaroons[, and CHERI token(s)]
class ResourceBase : public rclcpp::Node
{
public:
    ResourceBase(const std::string & authentication_topic, const std::string & command_topic, const std::string & node_name);

    void add_first_party_caveat(const std::string & first_party_caveat);
    void add_third_party_caveat(const std::string & location, const std::string & key, const std::string & identifier);

protected:
    void run(void);

    void initialise_discharge_macaroon(void);
    void print_resource_macaroon(void);
    void print_discharge_macaroon(void);
    std::vector<std::string> split_string(const std::string & input, const std::string & delimiter);
    std::string generate_key(std::size_t length);

    // Hold the name of the resource
    std::string resource_;

    // This macaroon is the token held by the node.
    macaroons::Macaroon resource_macaroon;

    // This is a Discharge macaroon that allows the holder to discharge a third party caveat
    macaroons::Macaroon discharge_macaroon;

    // pub/sub topic names
    std::string authentication_topic_;
    std::string command_topic_;

    rclcpp::TimerBase::SharedPtr timer_;

private:
    // hold the node name.  might not need this...
    std::string node_name_;

    // initial properties of the discharge caveat
    std::string discharge_key_;
    std::string discharge_location_;

    // // Subscribers
    rclcpp::Subscription<macaroon_msgs::msg::ResourceMacaroon>::SharedPtr resource_macaroon_sub_;
    rclcpp::Subscription<macaroon_msgs::msg::DischargeMacaroon>::SharedPtr discharge_macaroon_sub_;

    // callbacks
    void resource_macaroon_cb(const macaroon_msgs::msg::ResourceMacaroon::SharedPtr msg);
    void discharge_macaroon_cb(const macaroon_msgs::msg::DischargeMacaroon::SharedPtr msg);
};

#endif // RESOURCE_BASE_HPP