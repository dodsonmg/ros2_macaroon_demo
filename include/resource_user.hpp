#ifndef RESOURCE_USER_HPP
#define RESOURCE_USER_HPP

#include <chrono>
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
#include "macaroon_msgs/srv/initiate_tofu.hpp"
#include "macaroon_msgs/srv/authenticate.hpp"
#include "macaroon_msgs/srv/get_resource_token.hpp"
#include "macaroon_msgs/srv/use_resource_token.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

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
    void get_resource_token(void);
    void use_resource_token(const std::string & command);

    // macaroon interactions
    void add_first_party_caveat(const std::string & first_party_caveat);
    void add_third_party_caveat(const std::string & location, const std::string & key, const std::string & identifier);

protected:
    // initialisation
    void initialise_publishers(void);
    void initialise_subscribers(void);
    void initialise_clients(void);

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

    // service servers and clients
    rclcpp::Service<macaroon_msgs::srv::InitiateTofu>::SharedPtr tofu_server_;
    rclcpp::Client<macaroon_msgs::srv::InitiateTofu>::SharedPtr tofu_client_;

    rclcpp::Service<macaroon_msgs::srv::Authenticate>::SharedPtr authentication_server_;
    rclcpp::Client<macaroon_msgs::srv::Authenticate>::SharedPtr authentication_client_;

    rclcpp::Service<macaroon_msgs::srv::GetResourceToken>::SharedPtr get_resource_token_server_;
    rclcpp::Client<macaroon_msgs::srv::GetResourceToken>::SharedPtr get_resource_token_client_;

    rclcpp::Service<macaroon_msgs::srv::UseResourceToken>::SharedPtr use_resource_token_server_;
    rclcpp::Client<macaroon_msgs::srv::UseResourceToken>::SharedPtr use_resource_token_client_;

    // callbacks
    void initiate_tofu_cb(rclcpp::Client<macaroon_msgs::srv::InitiateTofu>::SharedFuture response);
    void initiate_authentication_cb(rclcpp::Client<macaroon_msgs::srv::Authenticate>::SharedFuture response);
    void get_resource_token_cb(rclcpp::Client<macaroon_msgs::srv::GetResourceToken>::SharedFuture response);
    void use_resource_token_cb(rclcpp::Client<macaroon_msgs::srv::UseResourceToken>::SharedFuture response);
};

#endif // RESOURCE_USER_HPP