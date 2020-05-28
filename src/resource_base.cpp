#include "resource_base.hpp"

ResourceBase::ResourceBase(const std::string & node_name, const std::string & authentication_topic, const std::string & command_topic)
: Node(node_name)
{
    node_name_ = node_name;

    // store topic information
    authentication_topic_ = authentication_topic;
    command_topic_ = command_topic;

    // initialise publisher for sending authentication and resource requests
    authentication_pub_ = this->create_publisher<macaroon_msgs::msg::MacaroonResourceRequest>(authentication_topic, 10);

    // initialise publisher for sending commands to the resource owner
    command_pub_ = this->create_publisher<macaroon_msgs::msg::MacaroonCommand>(command_topic_, 10);
    
    // initialise subscribers for receiving macaroons from the owner during initial authorisation
    resource_macaroon_sub_ = this->create_subscription<macaroon_msgs::msg::ResourceMacaroon>(
        authentication_topic, 10, std::bind(&ResourceBase::resource_macaroon_cb, this, _1));    

    discharge_macaroon_sub_ = this->create_subscription<macaroon_msgs::msg::DischargeMacaroon>(
        authentication_topic, 10, std::bind(&ResourceBase::discharge_macaroon_cb, this, _1));  
}

void
ResourceBase::run(void)
{
    /* might do something here */
}

// Initialise a discharge Macaroon.  Caveats added after.
// This function could be used by an owner, intermediary (e.g., third party) or user
void
ResourceBase::initialise_discharge_macaroon(const std::string location, const std::string key, const std::string identifier)
{
    D_.initialise(location, key, identifier);
}

// Add a first party caveat to the "owned" Macaroon
void
ResourceBase::add_first_party_caveat(const std::string first_party_caveat)
{
    M_.add_first_party_caveat(first_party_caveat);
}

// Add a third party caveat to the "owned" Macaroon
void
ResourceBase::add_third_party_caveat(const std::string location, const std::string key, const std::string identifier)
{
    M_.add_third_party_caveat(location, key, identifier);
}

// Send a request to a resource owner to initiate TOFU
void
ResourceBase::publish_authentication_request(const std::string resource)
{
    // create a key, a (unused) location, and assign the resource to the 'identifier' field
    TOFU_key_ = random_string(32);
    TOFU_location_ = "https://www.unused_third_party.com/";
    TOFU_identifier_ = resource;

    // publish the request
    auto msg = std::make_unique<macaroon_msgs::msg::MacaroonResourceRequest>();
    msg->key = TOFU_key_;
    msg->location = TOFU_location_;
    msg->identifier = TOFU_identifier_;
    RCLCPP_INFO(this->get_logger(), "Publishing resource request (key: %s, location: %s, identifier: %s)", 
        msg->key.c_str(), msg->location.c_str(), msg->identifier.c_str());

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    authentication_pub_->publish(std::move(msg));
}

// Send a command macaroon on the command topic
void
ResourceBase::publish_command(const std::string command)
{
    // create a temporary macaroon and add the command as a first party caveat
    Macaroon resource_macaroon = M_;
    resource_macaroon.add_first_party_caveat(command);

    // bind the discharge macaroon to the command macaroon
    Macaroon bound_discharge_macaroon = resource_macaroon.prepare_for_request(D_);

    // create a message with teh command and discharge macaroons
    auto msg = std::make_unique<macaroon_msgs::msg::MacaroonCommand>();
    msg->command_macaroon.macaroon = resource_macaroon.serialise();
    msg->discharge_macaroon.macaroon = bound_discharge_macaroon.serialise();

    // publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing command: %s", command.c_str());
    command_pub_->publish(std::move(msg));

    // print the macaroons for debug
    // resource_macaroon.print_macaroon();
    // bound_discharge_macaroon.print_macaroon();
}

void
ResourceBase::print_macaroon()
{
    M_.print_macaroon();
}

void
ResourceBase::print_discharge_macaroon()
{
    D_.print_macaroon();
}

// Create a callback function for when a resource macaroon is returned.
void
ResourceBase::resource_macaroon_cb(const macaroon_msgs::msg::ResourceMacaroon::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received resource macaroon");

    // Retrieve serialised resource macaroon from the message
    M_.deserialise(msg->resource_macaroon.macaroon);

    M_.print_macaroon();
}

// Create a callback function for when a discharge macaroon is returned.
void
ResourceBase::discharge_macaroon_cb(const macaroon_msgs::msg::DischargeMacaroon::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received discharge macaroon");

    // Retrieve serialised discharge macaroon from the message
    D_.deserialise(msg->discharge_macaroon.macaroon);

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