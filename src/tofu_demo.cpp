#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

/* macaroons */
#include "macaroons/macaroons.hpp"

// #include "resource_base.hpp"
#include "resource_owner.hpp"
#include "resource_user.hpp"

using namespace std::chrono_literals;

std::string separator = "--------------------------------------------------------------------------------";

int main(int argc, char * argv[])
{
    // Initialize any global resources needed by the middleware and the client library.
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // create single threaded executor to run both the publisher and subscriber
    // within the same process
    rclcpp::executors::SingleThreadedExecutor exec;

    // Establish the topics for authentication and command
    auto tofu_topic = std::string("tofu");
    auto authentication_topic = std::string("authentication");
    auto get_resource_topic = std::string("get_resource");
    auto use_resource_topic = std::string("use_resource");

    // instantiate nodes and spin a few times
    std::string resource_name = "cmd_vel";  // this is the resource owned or requested
    auto resource_owner = std::make_shared<ResourceOwner>(tofu_topic, authentication_topic, get_resource_topic, use_resource_topic, resource_name, resource_name + "_owner", false);
    auto resource_user = std::make_shared<ResourceUser>(tofu_topic, authentication_topic, get_resource_topic, use_resource_topic, resource_name, resource_name + "_user");

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << separator << std::endl;

    /**
     * User -> Owner:  initiate TOFU (request a key for a discharge macaroon)
     * Owner -> User:  if approved, send the key and add a third party caveat to the resource macaroon
     * */
    (*resource_user).initiate_tofu();

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << separator << std::endl;

    /**
     * User -> Owner:  add request to resource macaroon, bind the discharge macaroon, and send the request
     * NOTE:  This should fail, since we haven't added it as a valid command to the Owner yet
     * */
    std::string command = "command = speed_up";
    (*resource_user).use_resource_token(command);

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << separator << std::endl;

    /**
     * Owner:  Add the command as a valid command to the verifier
     * User -> Owner:  add request to resource macaroon, bind the discharge macaroon, and send the request
     * NOTE:  This should pass now, since we have added it as a valid command to the Owner
     * */
    (*resource_owner).add_valid_command_verifier(command);
    (*resource_user).use_resource_token(command);

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << separator << std::endl;

    rclcpp::shutdown();
    return 0;
}