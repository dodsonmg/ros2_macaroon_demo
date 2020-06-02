#include "resource_user.hpp"

ResourceUser::ResourceUser(const std::string & tofu_topic, const std::string & authentication_topic,
    const std::string & resource_topic, const std::string & resource_name, const std::string & node_name)
: Node(node_name)
{
    node_name_ = node_name;
    resource_name_ = resource_name;

    // store topic information
    tofu_topic_ = tofu_topic;
    authentication_topic_ = authentication_topic;
    resource_topic_ = resource_topic;

    initialise_clients();
}

void
ResourceUser::initialise_clients(void)
{
    tofu_client_ = this->create_client<macaroon_msgs::srv::InitiateTofu>(tofu_topic_);
    authentication_client_ = this->create_client<macaroon_msgs::srv::Authenticate>(authentication_topic_);
    get_resource_token_client_ = this->create_client<macaroon_msgs::srv::GetResourceToken>(resource_topic_);
    use_resource_token_client_ = this->create_client<macaroon_msgs::srv::UseResourceToken>(resource_topic_);
}

// Initialise a discharge macaroon.  Caveats added after.
// This function is used by the authoriser, who may be the owner or an intermediary
void
ResourceUser::initialise_discharge_macaroon(void)
{
    discharge_macaroon_ = macaroons::Macaroon("", discharge_macaroon_key_, resource_name_);
}

// Add a first party caveat to the resource macaroon
void
ResourceUser::add_first_party_caveat(const std::string & first_party_caveat)
{
    resource_macaroon_ = resource_macaroon_.add_first_party_caveat(first_party_caveat);
}

// Add a third party caveat to the resource macaroon
void
ResourceUser::add_third_party_caveat(const std::string & location, const std::string & key, const std::string & identifier)
{
    resource_macaroon_ = resource_macaroon_.add_third_party_caveat(location, key, identifier);
}

void
ResourceUser::print_resource_macaroon()
{
    std::cout << resource_macaroon_.inspect() << std::endl;
}

void
ResourceUser::print_discharge_macaroon()
{
    std::cout << discharge_macaroon_.inspect() << std::endl;
}

/**
 * Initiate TOFU service
 *
 * Request: resource name
 * Response: key for a discharge macaroon
 *
 * In a two party scenario, this is user->owner
 * In a three part scenario, this is third-party->owner
*/
void
ResourceUser::initiate_tofu(void)
{
    // publish the request
    auto request = std::make_shared<macaroon_msgs::srv::InitiateTofu::Request>();
    request->tofu_request = resource_name_;

    RCLCPP_INFO(this->get_logger(), "Initiating TOFU (request: %s)",
        request->tofu_request.c_str());

    // wait for service to be available
    while (!tofu_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        discharge_macaroon_key_ = "";
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // send the request
    auto result = tofu_client_->async_send_request(request, std::bind(&ResourceUser::initiate_tofu_cb, this, _1));
}

/**
 * TOFU service callback
 *
 * If TOFU was accepted, response is a key for a discharge macaroon.  Otherwise, empty string.
 * */
void
ResourceUser::initiate_tofu_cb(rclcpp::Client<macaroon_msgs::srv::InitiateTofu>::SharedFuture response)
{
    if(response.get()->key.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Received a TOFU response (key: %s)", response.get()->key.c_str());

        // Retrieve serialized resource macaroon from the message
        discharge_macaroon_key_ = response.get()->key;

        // initialise and store the discharge macaroon
        initialise_discharge_macaroon();
    } else {
        RCLCPP_INFO(this->get_logger(), "TOFU response failed");
        discharge_macaroon_key_ = "";
    }
}

/**
 * Initiate Authentication service
 *
 * Request: resource name and principal name
 * Response: serialised discharge macaroon
 * */
void
ResourceUser::initiate_authentication(void)
{
    // publish the request
    auto request = std::make_shared<macaroon_msgs::srv::Authenticate::Request>();

    request->principal_name = resource_name_ + "_user";
    request->resource_name = resource_name_;
    RCLCPP_INFO(this->get_logger(), "Initiating authentication (principal name: %s // resource name: %s)",
        request->principal_name.c_str(), request->resource_name.c_str());

    // wait for service to be available
    while (!authentication_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        discharge_macaroon_key_ = "";
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // send the request
    auto result = authentication_client_->async_send_request(request, std::bind(&ResourceUser::initiate_authentication_cb, this, _1));
}


/**
 * Authentication service callback
 *
 * If authentication was accepted, response is a serialised discharge macaroon.  Otherwise, empty string.
 * */
void
ResourceUser::initiate_authentication_cb(rclcpp::Client<macaroon_msgs::srv::Authenticate>::SharedFuture response)
{
    if(response.get()->discharge_macaroon.macaroon.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Received discharge macaroon");

        // Retrieve serialized resource macaroon from the message
        discharge_macaroon_ = macaroons::Macaroon::deserialize(response.get()->discharge_macaroon.macaroon);
    } else {
        RCLCPP_INFO(this->get_logger(), "Failed to receive a discharge macaroon");
    }
}

/**
 * Initiate GetResourceToken service
 *
 * Request: resource name
 * Response: serialised resource macaroon
 * */
void
ResourceUser::get_resource_token(void)
{
    // publish the request
    auto request = std::make_shared<macaroon_msgs::srv::GetResourceToken::Request>();

    request->resource_name = resource_name_;
    RCLCPP_INFO(this->get_logger(), "Requesting resource token (resource: %s)",
        request->resource_name.c_str());

    // wait for service to be available
    while (!get_resource_token_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        discharge_macaroon_key_ = "";
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // send the request
    auto result = get_resource_token_client_->async_send_request(request, std::bind(&ResourceUser::get_resource_token_cb, this, _1));
}

/**
 * GetResourceToken service callback
 *
 * If request was accepted, response is a serialised resource macaroon.  Otherwise, empty string.
 * */
void
ResourceUser::get_resource_token_cb(rclcpp::Client<macaroon_msgs::srv::GetResourceToken>::SharedFuture response)
{
    if(response.get()->resource_macaroon.macaroon.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Received resource macaroon");

        // Retrieve serialized resource macaroon from the message
        resource_macaroon_ = macaroons::Macaroon::deserialize(response.get()->resource_macaroon.macaroon);
    } else {
        RCLCPP_INFO(this->get_logger(), "Failed to receive a resource macaroon");
    }
}

/**
 * Initiate UseResourceToken service
 *
 * Request: resource macaroon caveated with a command, bound discharge macaroon
 * Response: successful execution
 * */
void
ResourceUser::use_resource_token(const std::string & command)
{
    // create a temporary macaroon and add the command as a first party caveat
    macaroons::Macaroon command_macaroon = resource_macaroon_;
    command_macaroon = command_macaroon.add_first_party_caveat(command);

    // bind the discharge macaroon to the command macaroon
    macaroons::Macaroon bound_discharge_macaroon = command_macaroon.prepare_for_request(discharge_macaroon_);

    // create a message with the command and discharge macaroons
    auto request = std::make_shared<macaroon_msgs::srv::UseResourceToken::Request>();
    request->command_macaroon.macaroon = command_macaroon.serialize();
    request->discharge_macaroon.macaroon = bound_discharge_macaroon.serialize();

    // publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing command: %s", command.c_str());

    // wait for service to be available
    while (!use_resource_token_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        discharge_macaroon_key_ = "";
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // send the request
    auto result = use_resource_token_client_->async_send_request(request, std::bind(&ResourceUser::use_resource_token_cb, this, _1));
}

/**
 * UseResourceToken service callback
 *
 * If command was executed, response is true, else false.
 * */
void
ResourceUser::use_resource_token_cb(rclcpp::Client<macaroon_msgs::srv::UseResourceToken>::SharedFuture response)
{
    if(response.get()->execution_success) {
        RCLCPP_INFO(this->get_logger(), "Command executed successfully.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Command failed to execute.");
    }
}

// used to split the output from Macaroon::inspect(), where "\n" is placed between caveats
std::vector<std::string>
ResourceUser::split_string(const std::string & input, const std::string & delimiter)
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
ResourceUser::generate_key(std::size_t length)
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