#include "resource_owner.hpp"

ResourceOwner::ResourceOwner(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic)
: ResourceBase(node_name, publish_topic, subscribe_topic)
{
    ResourceBase::timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ResourceOwner::run, this));
}

void
ResourceOwner::run(void)
{
    ResourceBase::run();
    verify_macaroon();
}

// Initialise the Macaroon, with or without caveats
// This function is only used by the Macaroon owner, who has the key
void
ResourceOwner::initialise_macaroon(const std::string location, const std::string key,
    const std::string identifier, const std::vector<std::string> first_party_caveats)
{
    ResourceBase::M_send_.initialise(location, key, identifier);
    if(first_party_caveats.size() > 0)
    {
        for (size_t i = 0; i < first_party_caveats.size(); i++)
        {
            ResourceBase::M_send_.add_first_party_caveat(first_party_caveats[i]);
        }
    }
}

// Initialise the MacaroonVerifier, with or without caveats
// This function is only used by the Macaroon owner, who has the key
void
ResourceOwner::initialise_verifier(const std::string key, const std::vector<std::string> first_party_caveats)
{
    V_.initialise(key);
    if(first_party_caveats.size() > 0)
    {
        for (size_t i = 0; i < first_party_caveats.size(); i++)
        {
            V_.satisfy_exact(first_party_caveats[i]);
        }
    }
}

void
ResourceOwner::add_first_party_caveats_macaroon(const std::vector<std::string> first_party_caveats)
{
    ResourceBase::add_first_party_caveats_macaroon(first_party_caveats);
}

// add first party caveats to the MacaroonVerifier
// This function is only used by the Macaroon owner
void
ResourceOwner::add_first_party_caveats_verifier(const std::vector<std::string> first_party_caveats)
{
    if(V_.initialised() && first_party_caveats.size() > 0)
    {
        for (size_t i = 0; i < first_party_caveats.size(); i++)
        {
            V_.satisfy_exact(first_party_caveats[i]);
        }
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
