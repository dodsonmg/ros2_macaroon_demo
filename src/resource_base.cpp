#include "resource_base.hpp"

ResourceBase::ResourceBase(const std::string & node_name, const std::string & publish_topic, const std::string & subscribe_topic, 
                           const std::string & authentication_topic)
: Node(node_name)
{
    node_name_ = node_name;

    T_ = std::make_shared<TalkerNode>(node_name + "_talker", publish_topic);
    L_ = std::make_shared<ListenerNode>(node_name + "_listener", subscribe_topic);

    // create a publisher for sending authentication and resource requests
    authentication_pub_ = this->create_publisher<macaroon_msgs::msg::MacaroonResourceRequest>(authentication_topic, 10);
    
    resource_and_discharge_macaroon_sub_ = this->create_subscription<macaroon_msgs::msg::DischargeMacaroons>(
        authentication_topic, 10, std::bind(&ResourceBase::resource_and_discharge_macaroons_cb, this, _1));    

    M_received_fresh_ = false;
    MS_received_fresh_ = false;
}

void
ResourceBase::run(void)
{
    // publish_macaroon();
    // exec_.spin_node_some(L_);  // allow the ListenerNode callback to execute
    // receive_macaroon();
}

// Initialise a discharge Macaroon.  Caveats added after.
// This function could be used by an owner, intermediary (e.g., third party) or user
void
ResourceBase::initialise_discharge_macaroon(const std::string location, const std::string key, const std::string identifier)
{
    D_.initialise(location, key, identifier);
}

// Add a first party caveat to the "owned" Macaroon by calling the Base class
void
ResourceBase::add_first_party_caveat(const std::string first_party_caveat)
{
    first_party_caveats_.push_back(first_party_caveat);
}

// Add a third party caveat to the "owned" Macaroon by calling the Base class
void
ResourceBase::add_third_party_caveat(const std::string location, const std::string key, const std::string identifier)
{
    ThirdPartyCaveat tpc = {location, key, identifier};
    third_party_caveats_.push_back(tpc);
}

// Adds first and third party caveats and publishes a serialised Macaroon
void
ResourceBase::publish_macaroon(void)
{
    if(M_.initialised())
    {
        // Derive a new Macaroon from M_ and add caveats
        Macaroon M_send = apply_caveats();

        // publish the serialised Macaroon with caveats
        (*T_).publish_message(M_send.serialise());

        std::vector<std::string> macaroons;
        macaroons.push_back(M_send.serialise());
        (*T_).publish_macaroons_message(macaroons);
    }
}

Macaroon
ResourceBase::apply_caveats(void)
{
    if(M_.initialised())
    {
        // Derive a new Macaroon from M_ and add caveats
        Macaroon M_applied = M_;
        for(std::string fpc : first_party_caveats_)
        {
            M_applied.add_first_party_caveat(fpc);
        }
        for(ThirdPartyCaveat tpc : third_party_caveats_)
        {
            M_applied.add_third_party_caveat(tpc.location, tpc.key, tpc.identifier);
        }

        return M_applied;
    }
    
    return NULL;
}

void
ResourceBase::receive_macaroon(void)
{
    std::string msg_received = (*L_).get_message();
    std::vector<std::string> macaroons_msg_received = (*L_).get_macaroons_message();
    if (msg_received.size() > 0)
    {
        M_received_.deserialise(msg_received);

        if(M_received_.initialised())
        {
            M_received_fresh_ = true;
        }
    }
    if (macaroons_msg_received.size() > 0)
    {
        for(std::string macaroon : macaroons_msg_received)
        {
            MS_received_.push_back(Macaroon(macaroon));
        }

        MS_received_fresh_ = true;

        for(Macaroon M : MS_received_)
        {
            if(!M.initialised())
            {
                MS_received_fresh_ = false;
            }
        }
    }
}

// Send a request to a resource owner to initiate TOFU
// TODO:  Possibly move the 'resource' to a private variable?  For this PoC, we're only dealing with one resource.
void
ResourceBase::authentication_and_resource_request(const std::string resource)
{
    // TODO: these should be initialised elsewhere.
    // TODO: this should generate a key and an id.  possibly the id should just be the resource??
    TOFU_key_ = random_string(32);
    TOFU_location_ = "https://www.unused_third_party.com/";
    TOFU_identifier_ = resource;
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

void
ResourceBase::print_macaroon()
{
    // Derive a new Macaroon from M_ and add caveats
    Macaroon M_applied = apply_caveats();
    M_applied.print_macaroon();
}

void
ResourceBase::print_discharge_macaroon()
{
    D_.print_macaroon();
}

// Create a callback function for when a resource and discharge macaroon are returned.
void
ResourceBase::resource_and_discharge_macaroons_cb(const macaroon_msgs::msg::DischargeMacaroons::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received resource/discharge macaroon pair");

    // Add serialised resource and discharge macaroons to the message
    M_received_.deserialise(msg->resource_macaroon.macaroon);
    D_.deserialise(msg->discharge_macaroon.macaroon);

    M_received_.print_macaroon();
    D_.print_macaroon();
}

/*
DANGER:  This came from GitHub...
https://github.com/InversePalindrome/Blog/tree/master/RandomString
*/
std::string
ResourceBase::random_string(std::size_t length)
{
    const std::string characters = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

    std::random_device random_device;
    std::mt19937 generator(random_device());
    std::uniform_int_distribution<> distribution(0, characters.size() - 1);

    std::string random_string;

    for (std::size_t i = 0; i < length; ++i)
    {
        random_string += characters[distribution(generator)];
    }

    return random_string;
}