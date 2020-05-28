#include "resource_user.hpp"

ResourceUser::ResourceUser(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic,
                           const std::string & authentication_topic, const std::string & command_topic)
: ResourceBase(node_name, publish_topic, subscribe_topic, authentication_topic, command_topic)
{
    ResourceBase::timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ResourceUser::run, this));
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