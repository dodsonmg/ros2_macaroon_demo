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
// void
// ResourceBase::publish_macaroon(void)
// {
//     if(M_.initialised())
//     {
//         // Derive a new Macaroon from M_ and add caveats
//         Macaroon M_send = apply_caveats();

//         // publish the serialised Macaroon with caveats
//         (*T_).publish_message(M_send.serialise());

//         std::vector<std::string> macaroons;
//         macaroons.push_back(M_send.serialise());
//         (*T_).publish_macaroons_message(macaroons);
//     }
// }

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

// void
// ResourceBase::receive_macaroon(void)
// {
//     std::string msg_received = (*L_).get_message();
//     std::vector<std::string> macaroons_msg_received = (*L_).get_macaroons_message();
//     if (msg_received.size() > 0)
//     {
//         M_received_.deserialise(msg_received);

//         if(M_received_.initialised())
//         {
//             M_received_fresh_ = true;
//         }
//     }
//     if (macaroons_msg_received.size() > 0)
//     {
//         for(std::string macaroon : macaroons_msg_received)
//         {
//             MS_received_.push_back(Macaroon(macaroon));
//         }

//         MS_received_fresh_ = true;

//         for(Macaroon M : MS_received_)
//         {
//             if(!M.initialised())
//             {
//                 MS_received_fresh_ = false;
//             }
//         }
//     }
// }

// Send a request to a resource owner to initiate TOFU
void
ResourceBase::authentication_and_resource_request(const std::string resource)
{
    // TODO: these should be initialised elsewhere.
    // TODO: this should generate a key and an id.  possibly the id should just be the resource??
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
    // create a temporary macaroon and apply all the current caveats
    Macaroon M_command = apply_caveats();

    // add the command as a first party caveat
    M_command.add_first_party_caveat(command);

    // bind the discharge macaroon to the command macaroon
    Macaroon D_bound = M_command.prepare_for_request(D_);

    // create a message with teh command and discharge macaroons
    auto msg = std::make_unique<macaroon_msgs::msg::MacaroonCommand>();
    msg->command_macaroon.macaroon = M_command.serialise();
    msg->discharge_macaroon.macaroon = D_bound.serialise();

    // publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing command: %s", command.c_str());
    command_pub_->publish(std::move(msg));

    // print the macaroons for debug
    M_command.print_macaroon();
    D_bound.print_macaroon();
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