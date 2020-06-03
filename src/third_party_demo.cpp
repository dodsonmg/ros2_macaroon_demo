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
#include "resource_authenticator.hpp"
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
    auto authenticator = std::make_shared<ResourceAuthenticator>(tofu_topic, authentication_topic, get_resource_topic, use_resource_topic, resource_name, resource_name + "_auth");
    auto resource_user = std::make_shared<ResourceUser>(tofu_topic, authentication_topic, get_resource_topic, use_resource_topic, resource_name, resource_name + "_user");

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(authenticator);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << separator << std::endl;

    /**
     * Auth -> Owner:  initiate TOFU (request a key for a discharge macaroon)
     * Owner -> Auth:  if approved, send the key and add a third party caveat to the resource macaroon
     * */
    (*authenticator).initiate_tofu();

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(authenticator);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << separator << std::endl;

    /**
     * User -> Auth:  request a resource macaroon
     * Auth -> User:  send a serialised, resource macaroon
     * */
    (*resource_user).get_resource_token();

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(authenticator);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << separator << std::endl;

    /**
     * User -> Auth:  initiate authentication (request a discharge macaroon and resource macaroon)
     * Auth -> User:  if approved, send a serialised discharge macaroon
     *
     * Unnecessary in the two party example, but we'll do it anyway for testing
     * */
    (*resource_user).initiate_authentication();

    // spin a bit
    for (int i = 1; i < 20; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(authenticator);
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
      exec.spin_node_some(authenticator);
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
      exec.spin_node_some(authenticator);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << separator << std::endl;

    return 0;

    // // Create a resource owner
    // // auto resource_owner = std::make_shared<ResourceOwner>("owner", issuer_topic, issuer_topic, authentication_topic);
    // (*resource_owner).initialise_macaroon(location, key, identifier);
    // (*resource_owner).initialise_verifier(key);

    // // Create and add first party caveats
    // std::string fpc_0 = "access = r/w/x";
    // std::string fpc_1 = "access = r";
    // (*resource_owner).add_first_party_caveat(fpc_0);  // adds fpc_0 to macaroon and verifier

    // // create third party caveat
    // std::string third_party_location = "https://www.unused_third_party.com/";
    // std::string third_party_key = "another_bad_key";
    // std::string third_party_predicate = "name = billyjoe";  // used by third party, not by the local verifier
    // // send_key_to_third_party(third_party_key, third_party_predicate)
    // // third_party_identifier = get_id_from_third_party()
    // std::string third_party_identifier = "another_bad_key_id";
    // (*resource_owner).add_third_party_caveat(third_party_location, third_party_key, third_party_identifier);

    // Create an intermediate resource user
    // auto resource_intermediary = std::make_shared<ResourceUser>("intermediary", intermediary_topic, issuer_topic, authentication_topic);

    // Create a resource user
    // auto resource_user = std::make_shared<ResourceUser>("user", user_topic, issuer_topic);

    // int max_iterations = 15;
    // for (int i = 0; i < max_iterations; i++)
    // {
    //   exec.spin_node_some(resource_owner);
    //   // exec.spin_node_some(resource_intermediary);
    //   // exec.spin_node_some(resource_user);

    //   // add a third party caveat to the Macaroon being sent
    //   if(i == (int)(max_iterations/3))
    //   {
    //     std::cout << std::endl << "<<< Adding third party caveat >>> " << std::endl;
    //     (*resource_owner).add_third_party_caveat(third_party_location, third_party_key, third_party_identifier);
    //     (*resource_owner).initialise_discharge_macaroon(third_party_location, third_party_key, third_party_identifier);
    //   }

    //   // // (a few iterations later...) add the first party caveat to the MacaroonVerifier
    //   // if(i == (int)(2*max_iterations/3))
    //   // {
    //   //   std::cout << std::endl << "<<< Adding MacaroonVerifier caveat to Owner >>> " << std::endl;
    //   //   (*resource_owner).add_first_party_caveat_verifier(fpc_1);
    //   // }

    //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    //   std::cout << std::endl;
    // }

    rclcpp::shutdown();
    return 0;
}