#include "resource_user.hpp"

ResourceUser::ResourceUser(const std::string & node_name, const std::string & authentication_topic, const std::string & command_topic)
: ResourceBase(node_name, authentication_topic, command_topic)
{
    ResourceBase::timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ResourceUser::run, this));
}

void
ResourceUser::run(void)
{
    ResourceBase::run();
}