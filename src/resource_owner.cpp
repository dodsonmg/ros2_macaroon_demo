#include "resource_owner.hpp"

ResourceOwner::ResourceOwner(const std::string & node_name, const std::string & authentication_topic, const std::string & command_topic,
                             const std::string location, const std::string identifier)
: ResourceBase(node_name, authentication_topic, command_topic)
{
    ResourceBase::timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ResourceOwner::run, this));

    // initialise the resource macaroon
    identifier_ = identifier;
    location_ = location;
    initialise_macaroon();

    // initialise the authentication subscriber
    authentication_sub_ = this->create_subscription<macaroon_msgs::msg::MacaroonResourceRequest>(
        ResourceBase::authentication_topic_, 10, std::bind(&ResourceOwner::authentication_and_resource_request_cb, this, _1));

    // initialise the command subscriber
    command_sub_ = this->create_subscription<macaroon_msgs::msg::MacaroonCommand>(
        ResourceBase::command_topic_, 10, std::bind(&ResourceOwner::command_cb, this, _1));
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
    ResourceBase::M_ = macaroons::Macaroon(location_, key_, identifier_);
}

// Add a first party caveat to the "owned" Macaroon by calling the Base class
// Add the same caveat to the MacaroonVerifier maintained by this derived class
void
ResourceOwner::add_first_party_caveat(const std::string first_party_caveat)
{
    ResourceBase::add_first_party_caveat(first_party_caveat);
    add_first_party_caveat_verifier(first_party_caveat);
}

// Add a possible commands (as first party caveats) to the MacaroonVerifier
void
ResourceOwner::add_valid_command(const std::string command)
{
    add_first_party_caveat_verifier(command);
}

// add first party caveats to the MacaroonVerifier
// This function is only used by the Macaroon owner
void
ResourceOwner::add_first_party_caveat_verifier(const std::string first_party_caveat)
{
    V_.satisfy_exact(first_party_caveat); 
}

bool
ResourceOwner::verify_macaroon(macaroons::Macaroon resource_macaroon, std::vector<macaroons::Macaroon> discharge_macaroons)
{
    // test the verification
    try {
        V_.verify(resource_macaroon, key_, discharge_macaroons);

        if(V_.verify_unsafe(resource_macaroon, key_, discharge_macaroons))
        {
            RCLCPP_INFO(this->get_logger(), "Macaroon verification: PASSED");
            return true;
        }        
    }
    catch (macaroons::exception::NotAuthorized &e) {
        std::cout << e.what() << std::endl;
    }

    RCLCPP_INFO(this->get_logger(), "Macaroon verification: FAILED");

    return false;
}

// Create a callback function for when resource access request messages are received.
void
ResourceOwner::authentication_and_resource_request_cb(const macaroon_msgs::msg::MacaroonResourceRequest::SharedPtr in_msg)
{
    RCLCPP_INFO(this->get_logger(), "Received resource request (key: %s, location: %s, identifier: %s)", 
        in_msg->key.c_str(), in_msg->location.c_str(), in_msg->identifier.c_str());

    // TODO:  check that the the resource is not currently in use

    // Create a discharge (TOFU) macaroon
    ResourceBase::initialise_discharge_macaroon(in_msg->location, in_msg->key, in_msg->identifier);

    // Add corresponding third party caveat to the resource macaroon
    ResourceBase::add_third_party_caveat(in_msg->location, in_msg->key, in_msg->identifier);

    publish_resource_and_discharge_macaroons();
}

// Create a callback function for when a command is received.
void
ResourceOwner::command_cb(const macaroon_msgs::msg::MacaroonCommand::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received command");

    // extract the command and discharge macaroons from the message
    macaroons::Macaroon command_macaroon(macaroons::Macaroon::deserialize(msg->command_macaroon.macaroon));
    macaroons::Macaroon discharge_macaroon(macaroons::Macaroon::deserialize(msg->discharge_macaroon.macaroon));

    // create a vector of macaroons to hold the discharge macaroon(s)
    std::vector<macaroons::Macaroon> discharge_macaroons = {discharge_macaroon};
    
    if(verify_macaroon(command_macaroon, discharge_macaroons)) {
        std::string command = extract_macaroon_command(command_macaroon);
        RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());
    }
}

// extracts a command from the command macaroon
// ASSUME:  there is only one command.  we should probably check for this...
std::string
ResourceOwner::extract_macaroon_command(macaroons::Macaroon command_macaroon)
{
    std::vector<std::string> caveats = ResourceBase::split_string(command_macaroon.inspect(), "\n");
    for (std::string s : caveats) {
        if(s.find("command =") != std::string::npos) {
            // the caveat will be something like "cid command = [command]"
            return s.substr(s.find("=") + 2);
        }
    }
    return "";
}

void
ResourceOwner::publish_resource_and_discharge_macaroons()
{
    // create an output message holding resource and discharge macaroons
    auto resource_msg = std::make_unique<macaroon_msgs::msg::ResourceMacaroon>();
    auto discharge_msg = std::make_unique<macaroon_msgs::msg::DischargeMacaroon>();
    
    // Add serialized resource and discharge macaroons to the message
    macaroons::Macaroon M_send = ResourceBase::M_;
    resource_msg->resource_macaroon.macaroon = M_send.serialize();
    discharge_msg->discharge_macaroon.macaroon = D_.serialize();

    // create a publisher and send the message
    resource_macaroon_pub_ = this->create_publisher<macaroon_msgs::msg::ResourceMacaroon>(ResourceBase::authentication_topic_, 10);
    resource_macaroon_pub_->publish(std::move(resource_msg));

    discharge_macaroon_pub_ = this->create_publisher<macaroon_msgs::msg::DischargeMacaroon>(ResourceBase::authentication_topic_, 10);
    discharge_macaroon_pub_->publish(std::move(discharge_msg));

    RCLCPP_INFO(this->get_logger(), "Published resource/discharge macaroon pair");
}