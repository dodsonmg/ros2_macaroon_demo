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
    auto attenuator_topic = std::string("attenuate_macaroon");
    auto user_topic = std::string("use_macaroon");

    // Create the owner's macaroon and serialise it
    std::string location = "https://www.unused.com/";
    std::string key = "a_bad_key";
    std::string identifier = "bad_key_id";
    std::string predicate = "access = r/w/x";
    Macaroon M_owner(location, key, identifier);
    Macaroon M_owner_received;
    M_owner.add_first_party_caveat(predicate);
    std::string M_owner_serialised = M_owner.serialise();
    std::string M_received_serialised;

    // Create and initialise the owner's MacaroonVerifier
    MacaroonVerifier V_owner(key);
    V_owner.satisfy_exact(predicate);

    // Create NULL macaroons for the intermediate node and user
    Macaroon M_intermediate;
    Macaroon M_user;

    // Create empty strings for serialised macaroons for intermediate and user
    std::string M_intermediate_serialised;
    std::string M_user_serialised;

    // Create a predicate for the intermediate to add
    std::string attenuation_predicate = "access = r";

    // Create a node for publishing the owner's macaroon and for receiving a macaroon from a user for verification.
    auto owner_talker = std::make_shared<TalkerNode>("owner_talker", issuer_topic);
    auto owner_listener = std::make_shared<ListenerNode>("owner_listener", user_topic);

    // Create a node for receiving a macaroon and for transmitting to the final user.
    auto intermediate_talker = std::make_shared<TalkerNode>("intermediate_talker", attenuator_topic);
    auto intermediate_listener = std::make_shared<ListenerNode>("intermediate_listener", issuer_topic);

    // Create a node for receiving a macaroon and transmitting it back to the owner
    auto user_talker = std::make_shared<TalkerNode>("user_talker", user_topic);
    auto user_listener = std::make_shared<ListenerNode>("user_listener", attenuator_topic);


    int max_loops = 10;
    for (int i = 0; i < max_loops; i++)
    {
      /**********************
      *  OWNER FUNCTIONS
      **********************/
      std::cout << std::endl << "<<< Perform owner functions >>>" << std::endl;
      (*owner_talker).put_message(M_owner_serialised);
      exec.spin_node_once(owner_talker);
      exec.spin_node_once(owner_listener);

      M_received_serialised = (*owner_listener).get_message();

      if(M_received_serialised.size() > 0)
      {
        // desearialise the received macaroon
        M_owner_received.deserialise(M_received_serialised);

        // verify the macaroon
        int result = V_owner.verify(M_owner_received);
        if (result == 0)
        {
            std::cout << "\t<<< MACAROON VERIFIED >>>" << std::endl;
        }
        else
        {
          std::cout << "\t<<< VERIFICATION FAILED: " << V_owner.get_verifier_error() << " >>>" << std::endl;
        }
        
        M_received_serialised = "";
      }

      // add the attenuation caveat partway through the loop, for demo purposes
      if(i == max_loops/2)
      {
        std::cout << "\t<<< Adding attenuation caveat >>>" << std::endl;
        V_owner.satisfy_exact(attenuation_predicate);
      }

      /************************
      *  INTERMEDIATE FUNCTIONS
      ************************/
      std::cout << std::endl << "<<< Perform intermediate functions >>>" << std::endl;
      exec.spin_node_once(intermediate_listener);  // listen for a published macaroon
      M_intermediate_serialised = (*intermediate_listener).get_message();  // get any serialised message

      // if we received a serialised macaroon, add a caveat and forward to the user
      if(M_intermediate_serialised.size() > 0)
      {
        // add a caveat to the macaroon before forwarding to the user
        M_intermediate.deserialise(M_intermediate_serialised);
        M_intermediate.add_first_party_caveat(attenuation_predicate);
        M_intermediate_serialised = M_intermediate.serialise();

        (*intermediate_talker).put_message(M_intermediate_serialised);
        exec.spin_node_once(intermediate_talker);
      }

      /**********************
      *  USER FUNCTIONS
      **********************/
      std::cout << std::endl << "<<< Perform user functions >>>" << std::endl;
      exec.spin_node_once(user_listener);  // listen for a published macaroon
      M_user_serialised = (*user_listener).get_message();  // get any serialised message

      // if we received a serialised macaroon, desearialise it, do something, then return it to owner
      if(M_user_serialised.size() > 0)
      {
        // M_user.deserialise(M_user_serialised);
        /* do something with the macaroon */
        // M_user_serialised = M_user.serialise();

        (*user_talker).put_message(M_user_serialised);
        exec.spin_node_once(user_talker);
      }
      
      /**********************
      *  SPIN
      **********************/      
      std::cout << std::endl << "<<< spinning... >>>" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
    return 0;
}