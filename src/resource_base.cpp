#include "resource_base.hpp"

ResourceBase::ResourceBase(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic)
: Node(node_name)
{
    T_ = std::make_shared<TalkerNode>(node_name + "_talker", publish_topic);
    L_ = std::make_shared<ListenerNode>(node_name + "_listener", subscribe_topic);

    M_received_fresh_ = false;
}

void
ResourceBase::run(void)
{
    publish_macaroon();
    exec_.spin_node_some(L_);
    receive_macaroon();
}

void
ResourceBase::add_first_party_caveats_macaroon(const std::vector<std::string> first_party_caveats)
{
    if(M_send_.initialised() && first_party_caveats.size() > 0)
    {
        for (size_t i = 0; i < first_party_caveats.size(); i++)
        {
            M_send_.add_first_party_caveat(first_party_caveats[i]);
        }
    }    
}

void
ResourceBase::publish_macaroon(void)
{
    if(M_send_.initialised())
    {
        (*T_).publish_message(M_send_.serialise());
    }
}

void
ResourceBase::receive_macaroon(void)
{
    std::string msg_received = (*L_).get_message();
    if (msg_received.size() > 0)
    {
        M_received_.deserialise(msg_received);
        M_received_fresh_ = true;
    }
}
