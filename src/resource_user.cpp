#include "resource_user.hpp"

ResourceUser::ResourceUser(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic)
: ResourceBase(node_name, publish_topic, subscribe_topic)
{
    ResourceBase::timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ResourceUser::run, this));
}

void
ResourceUser::run(void)
{
    ResourceBase::run();
    // If a Macaroon has been received and deserialised
    // Assign it to the send Macaroon to be serialised and published
    if(ResourceBase::M_received_.initialised() && ResourceBase::M_received_fresh_)
    {
        ResourceBase::M_send_ = ResourceBase::M_received_;
    }
}

// This is only necessary if the user is also an intermediate node
void
ResourceUser::add_first_party_caveats_macaroon(const std::vector<std::string> first_party_caveats)
{
    ResourceBase::add_first_party_caveats_macaroon(first_party_caveats);
}
