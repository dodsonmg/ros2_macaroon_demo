#ifndef RESOURCE_BASE_HPP
#define RESOURCE_BASE_HPP

// #include <chrono>

/* ROS2 */
#include "rclcpp/rclcpp.hpp"
#include "talker_node.hpp"
#include "listener_node.hpp"

/* macaroons */
#include "macaroons/macaroons.hpp"

// Create a Resource class that subclasses the generic rclcpp::Node base class.
// The Resource class encapsulates a publisher, subscriber, Macaroon[, and CHERI token]
class ResourceBase : public rclcpp::Node
{
public:
    ResourceBase(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic);

    void add_first_party_caveat(const std::string first_party_caveat = "");

protected:
    void run(void);
    void publish_macaroon(void);
    void receive_macaroon(void);

    // This is the Resource's "owned" Macaroon
    // i.e., M_ dictates the access this Resource is able to exercise and delegate
    Macaroon M_;

    // This is the Macaroon received from the ListenerNode
    // For resource intermediaries and users, it should be assigned to M_
    // For resource owners, it is separate from M_ and is verified by a MacaroonVerifier
    Macaroon M_received_;

    // This is set to true after receiving a valid Macaroon
    bool M_received_fresh_;

    rclcpp::TimerBase::SharedPtr timer_;

private:
    // create single threaded executor to run both the publisher and subscriber
    // within the same process
    rclcpp::executors::SingleThreadedExecutor exec_;

    // Define shared pointers to wrap the talker and listener nodes
    std::shared_ptr<TalkerNode> T_;
    std::shared_ptr<ListenerNode> L_;

    // Define a vector to hold first party caveats.  These will be applied against a macaroon before sending
    std::vector<std::string> first_party_caveats_;

    // TODO:  Populate these for received Macaroons if we are not the owner?
    std::string location_;
    std::string identifier_;


};

#endif // RESOURCE_BASE_HPP