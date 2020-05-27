#include "resource_user.hpp"

ResourceUser::ResourceUser(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic,
                           const std::string & authentication_topic)
: ResourceBase(node_name, publish_topic, subscribe_topic, authentication_topic)
{
    ResourceBase::timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ResourceUser::run, this));

    authentication_pub_ = this->create_publisher<macaroon_msgs::msg::MacaroonResourceRequest>(authentication_topic, 10);
}

void
ResourceUser::run(void)
{
    ResourceBase::run();

    // If a Macaroon has been received and deserialised
    // Assign it to the Resource's Macaroon
    if(ResourceBase::M_received_.initialised() && ResourceBase::M_received_fresh_)
    {
        ResourceBase::M_ = ResourceBase::M_received_;

        ResourceBase::M_received_fresh_ = false;
    }
}

// Add a first party caveat to the Resource's Macaroon by calling the Base class
// This is only exercised if the user is also an intermediate node
void
ResourceUser::add_first_party_caveat(const std::string first_party_caveat)
{
    ResourceBase::add_first_party_caveat(first_party_caveat);
}

// Send a request to a resource owner to initiate TOFU
// TODO:  Possibly move the 'resource' to a private variable?  For this PoC, we're only dealing with one resource.
void
ResourceUser::authentication_and_resource_request(const std::string resource)
{
    // TODO: these should be initialised elsewhere.
    // TODO: this should generate a key and an id.  possibly the id should just be the resource??
    TOFU_key_ = "another_bad_key:";
    TOFU_location_ = "https://www.unused_third_party.com/";
    TOFU_identifier_ = "another_bad_key_id";
    TOFU_resource_ = resource;

    // publish the request
    auto msg = std::make_unique<macaroon_msgs::msg::MacaroonResourceRequest>();
    msg->key = TOFU_key_;
    msg->location = TOFU_location_;
    msg->identifier = TOFU_identifier_;
    msg->resource = TOFU_resource_;
    RCLCPP_INFO(this->get_logger(), "Publishing resource request (key: %s, location: %s, identifier: %s, resource: %s)", 
        msg->key.c_str(), msg->location.c_str(), msg->identifier.c_str(), msg->resource.c_str());

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    authentication_pub_->publish(std::move(msg));
}
