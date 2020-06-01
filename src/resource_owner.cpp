#include "resource_owner.hpp"

ResourceOwner::ResourceOwner(const std::string & tofu_topic, const std::string & authentication_topic,
    const std::string & resource_topic, const std::string & resource_name, const std::string & node_name)
: ResourceUser(tofu_topic, authentication_topic, resource_topic, resource_name, node_name)
{
    // initialise the resource macaroon
    initialise_resource_macaroon();

    initialise_publishers();
    initialise_subscribers();
}

void
ResourceOwner::initialise_publishers(void)
{
    tofu_response_pub_ = this->create_publisher<macaroon_msgs::msg::TofuResponse>(tofu_topic_, 10);
    discharge_macaroon_pub_ = this->create_publisher<macaroon_msgs::msg::DischargeMacaroon>(authentication_topic_, 10);
    resource_macaroon_pub_ = this->create_publisher<macaroon_msgs::msg::ResourceMacaroon>(resource_topic_, 10);
}

void
ResourceOwner::initialise_subscribers(void)
{
    tofu_request_sub_ = this->create_subscription<macaroon_msgs::msg::TofuRequest>(
        tofu_topic_, 10, std::bind(&ResourceOwner::tofu_request_cb, this, _1));

    authentication_request_sub_ = this->create_subscription<macaroon_msgs::msg::AuthenticationRequest>(
        authentication_topic_, 10, std::bind(&ResourceOwner::authentication_request_cb, this, _1));

    resource_token_request_sub_ = this->create_subscription<macaroon_msgs::msg::ResourceTokenRequest>(
        resource_topic_, 10, std::bind(&ResourceOwner::resource_token_request_cb, this, _1));

    command_sub_ = this->create_subscription<macaroon_msgs::msg::CommandMacaroon>(
        resource_topic_, 10, std::bind(&ResourceOwner::command_cb, this, _1));
}

/**
 * Initialise the resource macaroon.
 *
 * This function is only used by the macaroon owner, who has the key
 */
void
ResourceOwner::initialise_resource_macaroon(void)
{
    resource_macaroon_key_ = ResourceUser::generate_key(32);
    resource_macaroon_ = macaroons::Macaroon("", resource_macaroon_key_, resource_name_);
}

/**
 * Add a first party caveat to the resource macaroon
 *
 * Add the same caveat to the verifier maintained by the owner
 */
void
ResourceOwner::add_first_party_caveat(const std::string & first_party_caveat)
{
    ResourceUser::add_first_party_caveat(first_party_caveat);
    add_first_party_caveat_verifier(first_party_caveat);
}

/**
 * Add a possible command (as a first party caveat) to the verifier
 */
void
ResourceOwner::add_valid_command_verifier(const std::string & command)
{
    add_first_party_caveat_verifier(command);
}

/**
 * add first party caveats to the verifier
 */
void
ResourceOwner::add_first_party_caveat_verifier(const std::string & first_party_caveat)
{
    V_.satisfy_exact(first_party_caveat);
}

bool
ResourceOwner::verify_macaroon(macaroons::Macaroon resource_macaroon, std::vector<macaroons::Macaroon> discharge_macaroons)
{
    // test the verification
    try {
        V_.verify(resource_macaroon, resource_macaroon_key_, discharge_macaroons);

        if(V_.verify_unsafe(resource_macaroon, resource_macaroon_key_, discharge_macaroons))
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

/**
 * Callback for receiving a TOFU request
 *
 * Response is either to transmit a key for use in a discharge macaroon or reject the request (empty string)
*/
void
ResourceOwner::tofu_request_cb(const macaroon_msgs::msg::TofuRequest::SharedPtr msg)
{
    auto tofu_response_msg = std::make_unique<macaroon_msgs::msg::TofuResponse>();

    // check that the TOFU resource is the resource owned by this node
    // if so, respond with a key, otherwise an empty string
    if(msg->tofu_request == resource_name_) {
        RCLCPP_INFO(this->get_logger(), "Valid TOFU request received (resource: %s).", msg->tofu_request.c_str());
        discharge_macaroon_key_ = generate_key(32);
        tofu_response_msg->key = discharge_macaroon_key_;

        // initialise the associated discharge macaroon
        initialise_discharge_macaroon();

        // Add corresponding third party caveat to the resource macaroon
        add_third_party_caveat("", discharge_macaroon_key_, resource_name_);
    } else {
        tofu_response_msg->key = "";
        RCLCPP_INFO(this->get_logger(), "TOFU request failed.  %s is an unknown resource.", msg->tofu_request.c_str());
    }

    tofu_response_pub_->publish(std::move(tofu_response_msg));
}

/**
 * Callback for receiving an authentication request
 *
 * Response is either to transmit a serialised discharge macaroon or reject the request (empty string)
 */
void
ResourceOwner::authentication_request_cb(const macaroon_msgs::msg::AuthenticationRequest::SharedPtr msg)
{
    auto discharge_macaroon_msg = std::make_unique<macaroon_msgs::msg::DischargeMacaroon>();

    // if the resource and principal are valid, respond with a serialised discharge macaroon
    if(msg->resource_name != resource_name_) {
        RCLCPP_INFO(this->get_logger(), "Authentication failed.  %s is an unknown resource.", msg->resource_name.c_str());
        discharge_macaroon_msg->discharge_macaroon.macaroon = "";
    } else if(msg->principal_name != resource_name_ + "_user") {
        RCLCPP_INFO(this->get_logger(), "Authentication failed.  %s is an unknown principal.", msg->principal_name.c_str());
        discharge_macaroon_msg->discharge_macaroon.macaroon = "";
    } else {
        RCLCPP_INFO(this->get_logger(), "Valid authentication request (resource: %s // principal: %s)",
            msg->resource_name.c_str(), msg->principal_name.c_str());
        // serialise the discharge macaroon and add it to the message
        discharge_macaroon_msg->discharge_macaroon.macaroon = discharge_macaroon_.serialize();
    }

    // send the discharge macaroon (or empty string)
    discharge_macaroon_pub_->publish(std::move(discharge_macaroon_msg));
}

/**
 * Callback for receiving a resource token request
 *
 * Response is either to transmit a serialised resource macaroon or reject the request (empty string)
 */
void
ResourceOwner::resource_token_request_cb(const macaroon_msgs::msg::ResourceTokenRequest::SharedPtr msg)
{
    auto resource_macaroon_msg = std::make_unique<macaroon_msgs::msg::ResourceMacaroon>();

    // if the resource and principal are valid, respond with a serialised discharge macaroon
    if(msg->resource_name != resource_name_) {
        RCLCPP_INFO(this->get_logger(), "Resource token request failed.  %s is an unknown resource.", msg->resource_name.c_str());
        resource_macaroon_msg->resource_macaroon.macaroon = "";
    } else {
        RCLCPP_INFO(this->get_logger(), "Valid resource token request received (resource: %s).", msg->resource_name.c_str());
        // serialise the resource macaroon and add it to the message
        resource_macaroon_msg->resource_macaroon.macaroon = resource_macaroon_.serialize();
    }

    // send the discharge macaroon (or empty string)
    resource_macaroon_pub_->publish(std::move(resource_macaroon_msg));
}

/*
 * Callback for receiving a command macaroon (caveated resource macaroon and bound discharge macaroon, both serialised)
 *
 * Extract, deserialise, and verify the command macaroon (with discharge macaroons).  If verified, extract (and execute) the command.
 */
void
ResourceOwner::command_cb(const macaroon_msgs::msg::CommandMacaroon::SharedPtr msg)
{
    // extract the command and discharge macaroons from the message
    macaroons::Macaroon command_macaroon(macaroons::Macaroon::deserialize(msg->command_macaroon.macaroon));
    macaroons::Macaroon bound_discharge_macaroon(macaroons::Macaroon::deserialize(msg->discharge_macaroon.macaroon));

    // create a vector of macaroons to hold the discharge macaroon(s)
    std::vector<macaroons::Macaroon> discharge_macaroons = {bound_discharge_macaroon};

    RCLCPP_INFO(this->get_logger(), "Received command macaroon");

    if(verify_macaroon(command_macaroon, discharge_macaroons)) {
        std::string command = extract_macaroon_command(command_macaroon);

        // perform some check that the command is acceptable
        RCLCPP_INFO(this->get_logger(), "Received valid command (command: %s).", command.c_str());
    }
}

/**
 * extracts a command from the command macaroon
 *
 * ASSUME:  there is only one command.  we should probably check for this...
 */
std::string
ResourceOwner::extract_macaroon_command(macaroons::Macaroon command_macaroon)
{
    std::vector<std::string> caveats = ResourceUser::split_string(command_macaroon.inspect(), "\n");
    for (std::string s : caveats) {
        if(s.find("command =") != std::string::npos) {
            // the caveat will be something like "cid command = [command]"
            return s.substr(s.find("=") + 2);
        }
    }
    return "";
}