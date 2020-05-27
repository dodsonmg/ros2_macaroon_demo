#include "resource_owner.hpp"

ResourceOwner::ResourceOwner(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic,
                             const std::string & authentication_topic)
: ResourceBase(node_name, publish_topic, subscribe_topic, authentication_topic)
{
    ResourceBase::timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ResourceOwner::run, this));

    authentication_sub_ = this->create_subscription<macaroon_msgs::msg::MacaroonResourceRequest>(
        authentication_topic, 10, std::bind(&ResourceOwner::authentication_and_resource_request_cb, this, _1));
}

void
ResourceOwner::run(void)
{
    ResourceBase::run();
    verify_macaroon();
}

// Initialise the Macaroon.  Caveats added after.
// This function is only used by the Macaroon owner, who has the key
void
ResourceOwner::initialise_macaroon(const std::string location, const std::string key, const std::string identifier)
{
    ResourceBase::M_.initialise(location, key, identifier);
}

// Initialise the MacaroonVerifier.  Caveats added after.
// This function is only used by the Macaroon owner, who has the key
void
ResourceOwner::initialise_verifier(const std::string key)
{
    V_.initialise(key);
}

// Add a first party caveat to the "owned" Macaroon by calling the Base class
// Add the same caveat to the MacaroonVerifier maintained by this derived class
void
ResourceOwner::add_first_party_caveat(const std::string first_party_caveat)
{
    ResourceBase::add_first_party_caveat(first_party_caveat);
    add_first_party_caveat_verifier(first_party_caveat);
}

// add first party caveats to the MacaroonVerifier
// This function is only used by the Macaroon owner
void
ResourceOwner::add_first_party_caveat_verifier(const std::string first_party_caveat)
{
    if(V_.initialised() && first_party_caveat.size() > 0)
    {
        V_.satisfy_exact(first_party_caveat);
    }    
}

bool
ResourceOwner::verify_macaroon(void)
{
    if(ResourceBase::M_received_.initialised() && ResourceBase::M_received_fresh_)
    {
        if(V_.verify(ResourceBase::M_received_))
        {
            RCLCPP_INFO(this->get_logger(), "Verification: PASSED");
            return true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Verification: FAILED (%s)", V_.get_verifier_error().c_str());
        }
        ResourceBase::M_received_fresh_ = false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Verification: No received Macaroons to verify yet...");
    }
    
    return false;
}

// Create a callback function for when resource access request messages are received.
void
ResourceOwner::authentication_and_resource_request_cb(const macaroon_msgs::msg::MacaroonResourceRequest::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Received resource request (key: %s, location: %s, identifier: %s, resource: %s)", 
        msg->key.c_str(), msg->location.c_str(), msg->identifier.c_str(), msg->resource.c_str());
}