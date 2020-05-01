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
    exec_.spin_node_some(L_);  // allow the ListenerNode callback to execute
    receive_macaroon();
}

// Add a first party caveat to the "owned" Macaroon by calling the Base class
void
ResourceBase::add_first_party_caveat(const std::string first_party_caveat)
{
    first_party_caveats_.push_back(first_party_caveat);
}

// Adds first party caveats and publishes a serialised Macaroon
void
ResourceBase::publish_macaroon(void)
{
    if(M_.initialised())
    {
        // Derive a new Macaroon from M_ and add caveats
        Macaroon M_send = M_;
        for(std::string caveat : first_party_caveats_)
        {
            M_send.add_first_party_caveat(caveat);
        }

        // publish the serialised Macaroon with caveats
        (*T_).publish_message(M_send.serialise());
    }
}

void
ResourceBase::receive_macaroon(void)
{
    std::string msg_received = (*L_).get_message();
    if (msg_received.size() > 0)
    {
        M_received_.deserialise(msg_received);

        if(M_received_.initialised())
        {
            M_received_fresh_ = true;
        }
    }
}
