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

    void add_first_party_caveats_macaroon(const std::vector<std::string> first_party_caveats = {});

protected:
    void run(void);
    void publish_macaroon(void);
    void receive_macaroon(void);

    Macaroon M_send_;
    Macaroon M_received_;
    bool M_received_fresh_;

    rclcpp::TimerBase::SharedPtr timer_;

private:
    // create single threaded executor to run both the publisher and subscriber
    // within the same process
    rclcpp::executors::SingleThreadedExecutor exec_;

    std::shared_ptr<TalkerNode> T_;
    std::shared_ptr<ListenerNode> L_;

    // TODO:  Populate these for received Macaroons if we are not the owner?
    std::string location_;
    std::string identifier_;


};

#endif // RESOURCE_BASE_HPP