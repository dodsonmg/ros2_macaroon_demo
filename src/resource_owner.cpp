#include "resource_owner.hpp"

ResourceOwner::ResourceOwner(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic,
                             const std::string & authentication_topic, const std::string location, const std::string identifier)
: ResourceBase(node_name, publish_topic, subscribe_topic, authentication_topic)
{
    ResourceBase::timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ResourceOwner::run, this));

    // store topic information
    ResourceBase::publish_topic_ = publish_topic;
    ResourceBase::subscribe_topic_ = subscribe_topic;
    ResourceBase::authentication_topic_ = authentication_topic;

    // initialise the resource macaroon
    identifier_ = identifier;
    location_ = location;
    initialise_macaroon();

    // initialise the authentication subscriber
    authentication_sub_ = this->create_subscription<macaroon_msgs::msg::MacaroonResourceRequest>(
        authentication_topic, 10, std::bind(&ResourceOwner::authentication_and_resource_request_cb, this, _1));
}

void
ResourceOwner::run(void)
{
    ResourceBase::run();
    // verify_macaroon();
}

// Initialise the Macaroon.  Caveats added after.
// This function is only used by the Macaroon owner, who has the key
void
ResourceOwner::initialise_macaroon(void)
{
    key_ = ResourceBase::random_string(32);
    ResourceBase::M_.initialise(location_, key_, identifier_);
}

// Initialise the MacaroonVerifier.  Caveats added after.
// This function is only used by the Macaroon owner, who has the key
void
ResourceOwner::initialise_verifier()
{
    V_.initialise(key_);
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
ResourceOwner::authentication_and_resource_request_cb(const macaroon_msgs::msg::MacaroonResourceRequest::SharedPtr in_msg)
{
    RCLCPP_INFO(this->get_logger(), "Received resource request (key: %s, location: %s, identifier: %s, resource: %s)", 
        in_msg->key.c_str(), in_msg->location.c_str(), in_msg->identifier.c_str(), in_msg->resource.c_str());

    // TODO:  check that the the resource is not currently in use

    // Create a discharge (TOFU) macaroon
    ResourceBase::initialise_discharge_macaroon(in_msg->location, in_msg->key, in_msg->identifier);
    ResourceBase::print_discharge_macaroon();

    // Add corresponding third party caveat to the resource macaroon
    ResourceBase::add_third_party_caveat(in_msg->location, in_msg->key, in_msg->identifier);
    ResourceBase::print_macaroon();

    publish_resource_and_discharge_macaroons();
}

void
ResourceOwner::publish_resource_and_discharge_macaroons()
{
    // create an output message holding resource and discharge macaroons
    auto out_msg = std::make_unique<macaroon_msgs::msg::DischargeMacaroons>();
    
    // Add serialised resource and discharge macaroons to the message
    Macaroon M_send = ResourceBase::apply_caveats();
    out_msg->resource_macaroon.macaroon = M_send.serialise();
    out_msg->discharge_macaroon.macaroon = D_.serialise();

    // create a publisher and send the message
    resource_and_discharge_macaroon_pub_ = this->create_publisher<macaroon_msgs::msg::DischargeMacaroons>(ResourceBase::authentication_topic_, 10);
    resource_and_discharge_macaroon_pub_->publish(std::move(out_msg));

    RCLCPP_INFO(this->get_logger(), "Published resource/discharge macaroon pair");
}