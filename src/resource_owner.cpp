#include "resource_owner.hpp"

ResourceOwner::ResourceOwner(const std::string & tofu_topic, const std::string & authentication_topic,
    const std::string & get_resource_topic, const std::string & use_resource_topic, const std::string & resource_name,
    const std::string & node_name, const bool two_party)
: ResourceUser(tofu_topic, authentication_topic, get_resource_topic, use_resource_topic, resource_name, node_name)
{
    // initialise the resource macaroon
    initialise_resource_macaroon();

    two_party_ = two_party;

    initialise_servers();
}

/**
 * Initialise servers run by this ResourceOwner
 * */
void
ResourceOwner::initialise_servers(void)
{
    tofu_server_ = this->create_service<macaroon_msgs::srv::InitiateTofu>(tofu_topic_,
        std::bind(&ResourceOwner::tofu_service, this, _1, _2));

    use_resource_token_server_ = this->create_service<macaroon_msgs::srv::UseResourceToken>(use_resource_topic_,
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

    // RCLCPP_INFO(this->get_logger(), "Macaroon verification: FAILED");

    return false;
}

/**
 * TOFU Service
 *
 * Receives a request to initiate TOFU
 * Responds with the resource macaroon and a key (for a discharge macaroon) or a rejection (empty strings)
 * */
void
ResourceOwner::tofu_service(const std::shared_ptr<macaroon_msgs::srv::InitiateTofu::Request> request,
    std::shared_ptr<macaroon_msgs::srv::InitiateTofu::Response> response)
{
    // check that the TOFU resource is the resource owned by this node
    // if so, respond with  the resource macaroon and a discharge macaroon key, otherwise an empty strings
    if(request->tofu_request == resource_name_) {
        discharge_macaroon_key_ = generate_key(32);
        response->key = discharge_macaroon_key_;

        // initialise the associated discharge macaroon
        initialise_discharge_macaroon();

        // Add corresponding third party caveat to the resource macaroon
        add_third_party_caveat("", discharge_macaroon_key_, resource_name_);

        // serialise the resource macaroon and add it to the message
        response->resource_macaroon.macaroon = resource_macaroon_.serialize();

        RCLCPP_INFO(this->get_logger(), "TOFU:\t\tValid request");
        RCLCPP_INFO(this->get_logger(), "> resource:\t%s", request->tofu_request.c_str());
        RCLCPP_INFO(this->get_logger(), "> discharge macaroon key:\t%s", discharge_macaroon_key_.c_str());
        RCLCPP_INFO(this->get_logger(), "> resource macaroon id:\t\t%s", resource_macaroon_.identifier().c_str());
        RCLCPP_INFO(this->get_logger(), "> resource macaroon signature:\t%s", resource_macaroon_.signature_string().c_str());
    } else {
        response->key = "";
        response->resource_macaroon.macaroon = "";
        RCLCPP_INFO(this->get_logger(), "TOFU:\t\tInvalid request");
        RCLCPP_INFO(this->get_logger(), "> resource:\t%s is unknown.", request->tofu_request.c_str());
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

    if(verify_macaroon(command_macaroon, discharge_macaroons)) {
        std::string command = extract_macaroon_command(command_macaroon);

        /* perform some check that the command is acceptable */

        response->execution_success = true;

        RCLCPP_INFO(this->get_logger(), "Token use:\tValid request");
        RCLCPP_INFO(this->get_logger(), "> command:\t%s", command.c_str());
        RCLCPP_INFO(this->get_logger(), "> resource macaroon id:\t\t%s", command_macaroon.identifier().c_str());
        RCLCPP_INFO(this->get_logger(), "> resource macaroon signature:\t%s", command_macaroon.signature_string().c_str());
        RCLCPP_INFO(this->get_logger(), "> discharge macaroon id:\t%s", bound_discharge_macaroon.identifier().c_str());
        RCLCPP_INFO(this->get_logger(), "> discharge macaroon signature:\t%s", bound_discharge_macaroon.signature_string().c_str());
    } else {
        response->execution_success = false;

        RCLCPP_INFO(this->get_logger(), "Token use:\tInvalid request");
        RCLCPP_INFO(this->get_logger(), "> resource macaroon id:\t\t%s", command_macaroon.identifier().c_str());
        RCLCPP_INFO(this->get_logger(), "> resource macaroon signature:\t%s", command_macaroon.signature_string().c_str());
        RCLCPP_INFO(this->get_logger(), "> discharge macaroon id:\t%s", bound_discharge_macaroon.identifier().c_str());
        RCLCPP_INFO(this->get_logger(), "> discharge macaroon signature:\t%s", bound_discharge_macaroon.signature_string().c_str());
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