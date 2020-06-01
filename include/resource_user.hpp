#ifndef RESOURCE_USER_HPP
#define RESOURCE_USER_HPP

// #include <chrono>
#include <random>

/* macaroons */
#include "macaroons/macaroons.hpp"

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "macaroon_msgs/msg/tofu_request.hpp"
#include "macaroon_msgs/msg/tofu_response.hpp"
#include "macaroon_msgs/msg/authentication_request.hpp"
#include "macaroon_msgs/msg/discharge_macaroon.hpp"
#include "macaroon_msgs/msg/resource_token_request.hpp"
#include "macaroon_msgs/msg/resource_macaroon.hpp"
#include "macaroon_msgs/msg/command_macaroon.hpp"

using std::placeholders::_1;

// Create a Resource class that subclasses the generic rclcpp::Node base class.
// The Resource class encapsulates one or more Macaroons[, and CHERI token(s)]
class ResourceUser : public rclcpp::Node
{
public:
    ResourceUser(const std::string & tofu_topic, const std::string & authentication_topic,
        const std::string & resource_topic, const std::string & resource_name, const std::string & node_name);

    // interactions between owner and user
    void initiate_tofu(void);
    void initiate_authentication(void);
    void request_resource_token(void);
    void transmit_command(const std::string & command);

    // macaroon interactions
    void add_first_party_caveat(const std::string & first_party_caveat);
    void add_third_party_caveat(const std::string & location, const std::string & key, const std::string & identifier);

protected:
    void run(void);

    // initialisation
    void initialise_publishers(void);
    void initialise_subscribers(void);

    // macaroon interactions
    void initialise_resource_macaroon(void);
    void initialise_discharge_macaroon(void);
    void print_resource_macaroon(void);
    void print_discharge_macaroon(void);

    // helper functions
    std::vector<std::string> split_string(const std::string & input, const std::string & delimiter);
    std::string generate_key(std::size_t length);

    // Hold the names of the resource and node
    std::string resource_name_;
    std::string node_name_;

    // This macaroon is the token held by the node.
    macaroons::Macaroon resource_macaroon_;
    std::string resource_macaroon_key_;

    // This is a discharge macaroon that allows the holder to discharge a third party caveat
    // Two party scenario:  created by the resource user
    // Three party scenario:  created by the third-party and transmitted to an authenticated resource user
    macaroons::Macaroon discharge_macaroon_;
    std::string discharge_macaroon_key_;

    // pub/sub topic names
    std::string tofu_topic_;
    std::string authentication_topic_;
    std::string resource_topic_;

    // publishers and subscribers (nominally in the order they would be used)
    rclcpp::Publisher<macaroon_msgs::msg::TofuRequest>::SharedPtr tofu_request_pub_;        // publish a request for a discharge macaroon key (assume this is a TOFU interaction)
    rclcpp::Subscription<macaroon_msgs::msg::TofuRequest>::SharedPtr tofu_request_sub_;     // receive...

    rclcpp::Publisher<macaroon_msgs::msg::TofuResponse>::SharedPtr tofu_response_pub_;      // publish a a discharge macaroon key (assume this is a TOFU interaction)
    rclcpp::Subscription<macaroon_msgs::msg::TofuResponse>::SharedPtr tofu_response_sub_;   // receive...

    rclcpp::Publisher<macaroon_msgs::msg::AuthenticationRequest>::SharedPtr authentication_request_pub_;      // publish an authentication request
    rclcpp::Subscription<macaroon_msgs::msg::AuthenticationRequest>::SharedPtr authentication_request_sub_;   // receive...

    rclcpp::Publisher<macaroon_msgs::msg::DischargeMacaroon>::SharedPtr discharge_macaroon_pub_;        // publish a serialised discharge macaroon (in response to an authentication request)
    rclcpp::Subscription<macaroon_msgs::msg::DischargeMacaroon>::SharedPtr discharge_macaroon_sub_;     // receive...

    rclcpp::Publisher<macaroon_msgs::msg::ResourceTokenRequest>::SharedPtr resource_token_request_pub_;      // publish a request for a resource token
    rclcpp::Subscription<macaroon_msgs::msg::ResourceTokenRequest>::SharedPtr resource_token_request_sub_;   // receive...

    rclcpp::Publisher<macaroon_msgs::msg::ResourceMacaroon>::SharedPtr resource_macaroon_pub_;      // publish a serialised resource macaroon (in response to a resource token request)
    rclcpp::Subscription<macaroon_msgs::msg::ResourceMacaroon>::SharedPtr resource_macaroon_sub_;   // receive...

    rclcpp::Publisher<macaroon_msgs::msg::CommandMacaroon>::SharedPtr command_pub_;     // publish a command (a serialised, caveated resource macaroon and a serialised, bound discharge macaroon)
    rclcpp::Subscription<macaroon_msgs::msg::CommandMacaroon>::SharedPtr command_sub_;  // receive...

    // callbacks
    void tofu_response_cb(const macaroon_msgs::msg::TofuResponse::SharedPtr msg);                       // receive a tofu response by storing a key (later used to create a discharge macaroon)
    void discharge_macaroon_cb(const macaroon_msgs::msg::DischargeMacaroon::SharedPtr msg);             // receive a discharge macaroon, deserialise it, and store it
    void resource_macaroon_cb(const macaroon_msgs::msg::ResourceMacaroon::SharedPtr msg);               // receive a resource macaroon, deserialise it, and store it
};

#endif // RESOURCE_USER_HPP