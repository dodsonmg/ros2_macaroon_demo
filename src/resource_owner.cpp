#include "resource_owner.hpp"

ResourceOwner::ResourceOwner(const std::string & tofu_topic, const std::string & authentication_topic,
    const std::string & resource_topic, const std::string & resource_name, const std::string & node_name)
: ResourceUser(tofu_topic, authentication_topic, resource_topic, resource_name, node_name)
{
    // initialise the resource macaroon
    initialise_resource_macaroon();

    initialise_servers();
}

void
ResourceOwner::initialise_servers(void)
{
    tofu_server_ = this->create_service<macaroon_msgs::srv::InitiateTofu>(tofu_topic_,
        std::bind(&ResourceOwner::tofu_service, this, _1, _2));
    authentication_server_ = this->create_service<macaroon_msgs::srv::Authenticate>(authentication_topic_,
        std::bind(&ResourceOwner::authentication_service, this, _1, _2));
    get_resource_token_server_ = this->create_service<macaroon_msgs::srv::GetResourceToken>(resource_topic_,
        std::bind(&ResourceOwner::get_resource_token_service, this, _1, _2));
    use_resource_token_server_ = this->create_service<macaroon_msgs::srv::UseResourceToken>(resource_topic_,
        std::bind(&ResourceOwner::use_resource_token_service, this, _1, _2));
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
 * TOFU Service
 *
 * Receives a request to initiate TOFU and responds with a key (for a discharge macaroon)
 * or a rejection (empty string)
 * */
void
ResourceOwner::tofu_service(const std::shared_ptr<macaroon_msgs::srv::InitiateTofu::Request> request,
    std::shared_ptr<macaroon_msgs::srv::InitiateTofu::Response> response)
{
    // check that the TOFU resource is the resource owned by this node
    // if so, respond with a key, otherwise an empty string
    if(request->tofu_request == resource_name_) {
        RCLCPP_INFO(this->get_logger(), "Valid TOFU request received (resource: %s).", request->tofu_request.c_str());
        discharge_macaroon_key_ = generate_key(32);
        response->key = discharge_macaroon_key_;

        // initialise the associated discharge macaroon
        initialise_discharge_macaroon();

        // Add corresponding third party caveat to the resource macaroon
        add_third_party_caveat("", discharge_macaroon_key_, resource_name_);
    } else {
        response->key = "";
        RCLCPP_INFO(this->get_logger(), "TOFU request failed.  %s is an unknown resource.", request->tofu_request.c_str());
    }
}

/**
 * Authentication service
 *
 * Receives an authentication request.
 * Responds with either a serialised discharge macaroon or rejection (empty string)
 */
void
ResourceOwner::authentication_service(const std::shared_ptr<macaroon_msgs::srv::Authenticate::Request> request,
    std::shared_ptr<macaroon_msgs::srv::Authenticate::Response> response)
{
    // if the resource and principal are valid, respond with a serialised discharge macaroon
    if(request->resource_name != resource_name_) {
        RCLCPP_INFO(this->get_logger(), "Authentication failed.  %s is an unknown resource.", request->resource_name.c_str());
        response->discharge_macaroon.macaroon = "";
    } else if(request->principal_name != resource_name_ + "_user") {
        RCLCPP_INFO(this->get_logger(), "Authentication failed.  %s is an unknown principal.", request->principal_name.c_str());
        response->discharge_macaroon.macaroon = "";
    } else {
        RCLCPP_INFO(this->get_logger(), "Valid authentication request (resource: %s // principal: %s)",
            request->resource_name.c_str(), request->principal_name.c_str());
        // serialise the discharge macaroon and add it to the message
        response->discharge_macaroon.macaroon = discharge_macaroon_.serialize();
    }
}

/**
 * GetResourceToken service
 *
 * Receives an resource request.
 * Responds with either a serialised resource macaroon or rejection (empty string)
 */
void
ResourceOwner::get_resource_token_service(const std::shared_ptr<macaroon_msgs::srv::GetResourceToken::Request> request,
    std::shared_ptr<macaroon_msgs::srv::GetResourceToken::Response> response)
{
    // if the resource and principal are valid, respond with a serialised discharge macaroon
    if(request->resource_name != resource_name_) {
        RCLCPP_INFO(this->get_logger(), "Resource token request failed.  %s is an unknown resource.", request->resource_name.c_str());
        response->resource_macaroon.macaroon = "";
    } else {
        RCLCPP_INFO(this->get_logger(), "Valid resource token request received (resource: %s)",
            request->resource_name.c_str());
        // serialise the discharge macaroon and add it to the message
        response->resource_macaroon.macaroon = resource_macaroon_.serialize();
    }
}

/**
 * UseResourceToken service
 *
 * Receives a command macaroon (caveated resource macaroon and bound discharge macaroon, both serialised)
 * Responds with true if the command is valid and executed correctly, false otherwise
 */
void
ResourceOwner::use_resource_token_service(const std::shared_ptr<macaroon_msgs::srv::UseResourceToken::Request> request,
    std::shared_ptr<macaroon_msgs::srv::UseResourceToken::Response> response)
{
    // extract the command and discharge macaroons from the message
    macaroons::Macaroon command_macaroon(macaroons::Macaroon::deserialize(request->command_macaroon.macaroon));
    macaroons::Macaroon bound_discharge_macaroon(macaroons::Macaroon::deserialize(request->discharge_macaroon.macaroon));

    // create a vector of macaroons to hold the discharge macaroon(s)
    std::vector<macaroons::Macaroon> discharge_macaroons = {bound_discharge_macaroon};

    RCLCPP_INFO(this->get_logger(), "Received command macaroon");

    if(verify_macaroon(command_macaroon, discharge_macaroons)) {
        std::string command = extract_macaroon_command(command_macaroon);

        /* perform some check that the command is acceptable */

        RCLCPP_INFO(this->get_logger(), "Received valid command (command: %s).", command.c_str());
        response->execution_success = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Received invalid command macaroon.");
        response->execution_success = false;
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