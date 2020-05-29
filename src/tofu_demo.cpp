#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "talker_node.hpp"
#include "listener_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"

/* macaroons */
#include "macaroons/macaroons.hpp"

// #include "resource_base.hpp"
#include "resource_owner.hpp"
#include "resource_user.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources needed by the middleware and the client library.
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // create single threaded executor to run both the publisher and subscriber
    // within the same process
    rclcpp::executors::SingleThreadedExecutor exec;

    // Establish the topics for authentication and command
    auto authentication_topic = std::string("authentication");
    auto command_topic = std::string("command");

    // instantiate nodes and spin a few times
    std::string location = "https://www.unused.com/";
    std::string identifier = "cmd_vel";  // this is the resource owned or requested
    auto resource_owner = std::make_shared<ResourceOwner>("owner", authentication_topic, command_topic, location, identifier);
    auto resource_user = std::make_shared<ResourceUser>("user", authentication_topic, command_topic);

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // User -> Owner:  request "access" via message (provide key and id)
    // Owner -> User:  if approved, send discharge macaroon and resource macaroon with appropriate third party caveat
    std::string resource = "cmd_vel";
    (*resource_user).publish_authentication_request(resource);
    
    // spin a bit
    for (int i = 1; i < 20; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // User -> Owner:  add request to resource macaroon, bind the discharge macaroon, and send the request
    // NOTE:  This should fail, since we haven't added it as a valid command to the Owner yet
    std::string command = "command = speed_up";
    (*resource_user).publish_command(command);

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Owner:  Add the command as a valid command to the verifier
    (*resource_owner).add_valid_command(command);

    // User -> Owner:  add request to resource macaroon, bind the discharge macaroon, and send the request
    // NOTE:  This should pass now, since we have added it as a valid command to the Owner
    (*resource_user).publish_command(command);

    // spin a bit
    for (int i = 1; i < 10; ++i)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(resource_user);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Owner:  extract request from resource macaroon (this is part of the callback...)




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