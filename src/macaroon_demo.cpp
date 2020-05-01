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

    // Establish the topics for delegating a macaroon and receiving one for verification.
    // A -> <issuer_topic> -> B -> <attenuator_topic> -> C -> <user_topic> -> A
    auto issuer_topic = std::string("issue_macaroon");
    auto intermediary_topic = std::string("attenuate_macaroon");
    auto user_topic = std::string("use_macaroon");

    // Create the owner's macaroon and serialise it
    std::string location = "https://www.unused.com/";
    std::string key = "a_bad_key";
    std::string identifier = "bad_key_id";
    std::string fpc_0 = "access = r/w/x";
    std::string fpc_1 = "access = r";

    // Create a resource owner
    auto resource_owner = std::make_shared<ResourceOwner>("owner", issuer_topic, user_topic);
    (*resource_owner).initialise_macaroon(location, key, identifier);
    (*resource_owner).initialise_verifier(key);
    (*resource_owner).add_first_party_caveat(fpc_0);  // adds to macaroon and verifier

    // Create an intermediate resource user
    auto resource_intermediary = std::make_shared<ResourceUser>("intermediary", intermediary_topic, issuer_topic);

    // Create a resource user
    auto resource_user = std::make_shared<ResourceUser>("user", user_topic, intermediary_topic);

    int max_iterations = 15;
    for (int i = 0; i < max_iterations; i++)
    {
      exec.spin_node_some(resource_owner);
      exec.spin_node_some(resource_intermediary);
      exec.spin_node_some(resource_user);

      // add a third party caveat to the Macaroon being sent
      if(i == (int)(max_iterations/3))
      {
        std::cout << std::endl << "<<< Adding Macaroon caveat to Intermediary >>> " << std::endl;
        (*resource_intermediary).add_first_party_caveat(fpc_1);
      }

      // (a few iterations later...) add the third party caveat to the MacaroonVerifier
      if(i == (int)(2*max_iterations/3))
      {
        std::cout << std::endl << "<<< Adding MacaroonVerifier caveat to Owner >>> " << std::endl;
        (*resource_owner).add_first_party_caveat_verifier(fpc_1);
      }      

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      std::cout << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}