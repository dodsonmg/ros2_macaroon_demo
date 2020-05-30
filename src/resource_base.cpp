#include "resource_base.hpp"

ResourceBase::ResourceBase(const std::string & authentication_topic, const std::string & command_topic, const std::string & node_name)
: Node(node_name)
{
    node_name_ = node_name;

    // store topic information
    authentication_topic_ = authentication_topic;
    command_topic_ = command_topic;
    
    // initialise subscribers for receiving macaroons from the owner during initial authorisation
    resource_macaroon_sub_ = this->create_subscription<macaroon_msgs::msg::ResourceMacaroon>(
        authentication_topic, 10, std::bind(&ResourceBase::resource_macaroon_cb, this, _1));    

    discharge_macaroon_sub_ = this->create_subscription<macaroon_msgs::msg::DischargeMacaroon>(
        authentication_topic, 10, std::bind(&ResourceBase::discharge_macaroon_cb, this, _1));  
}

void
ResourceBase::run(void)
{
    /* might do something here. called by exec during a spin(). */
}

// Initialise a discharge macaroon.  Caveats added after.
// This function is used by the authoriser, who may be the owner or an intermediary
void
ResourceBase::initialise_discharge_macaroon(void)
{
    discharge_key_ = generate_key(32);
    discharge_macaroon = macaroons::Macaroon("", discharge_key_, resource_);

    // Add corresponding third party caveat to the resource macaroon
    add_third_party_caveat("", discharge_key_, resource_);
}

// Add a first party caveat to the resource macaroon
void
ResourceBase::add_first_party_caveat(const std::string & first_party_caveat)
{
    resource_macaroon = resource_macaroon.add_first_party_caveat(first_party_caveat);
}

// Add a third party caveat to the resource macaroon
void
ResourceBase::add_third_party_caveat(const std::string & location, const std::string & key, const std::string & identifier)
{
    resource_macaroon = resource_macaroon.add_third_party_caveat(location, key, identifier);
}

void
ResourceBase::print_resource_macaroon()
{
    std::cout << resource_macaroon.inspect() << std::endl;
}

void
ResourceBase::print_discharge_macaroon()
{
    std::cout << discharge_macaroon.inspect() << std::endl;
}

// Create a callback function for when a resource macaroon is returned.
void
ResourceBase::resource_macaroon_cb(const macaroon_msgs::msg::ResourceMacaroon::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received resource macaroon");

    // Retrieve serialized resource macaroon from the message
    resource_macaroon = macaroons::Macaroon::deserialize(msg->resource_macaroon.macaroon);
}

// Create a callback function for when a discharge macaroon is returned.
void
ResourceBase::discharge_macaroon_cb(const macaroon_msgs::msg::DischargeMacaroon::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received discharge macaroon");

    // Retrieve serialized discharge macaroon from the message
    discharge_macaroon = macaroons::Macaroon::deserialize(msg->discharge_macaroon.macaroon);
}

// used to split the output from Macaroon::inspect(), where "\n" is placed between caveats
std::vector<std::string>
ResourceBase::split_string(const std::string & input, const std::string & delimiter)
{
    std::vector<std::string> output;
    std::string::size_type last = 0;
    std::string::size_type next = 0;

    while ((next = input.find(delimiter, last)) != std::string::npos) {
        output.push_back(input.substr(last, next-last));
        last = next + 1;
    }

    output.push_back(input.substr(last, next-last));

    return output;
}

/*
DANGER:  This came from GitHub...
https://github.com/InversePalindrome/Blog/tree/master/RandomString
*/
std::string
ResourceBase::generate_key(std::size_t length)
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