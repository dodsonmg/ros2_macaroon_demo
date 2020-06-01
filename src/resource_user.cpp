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

    initialise_publishers();
    initialise_subscribers();
}

void
ResourceUser::initialise_publishers(void)
{
    tofu_request_pub_ = this->create_publisher<macaroon_msgs::msg::TofuRequest>(tofu_topic_, 10);
    authentication_request_pub_ = this->create_publisher<macaroon_msgs::msg::AuthenticationRequest>(authentication_topic_, 10);
    resource_token_request_pub_ = this->create_publisher<macaroon_msgs::msg::ResourceTokenRequest>(resource_topic_, 10);
    command_pub_ = this->create_publisher<macaroon_msgs::msg::CommandMacaroon>(resource_topic_, 10);
}

void
ResourceUser::initialise_subscribers(void)
{
    tofu_response_sub_ = this->create_subscription<macaroon_msgs::msg::TofuResponse>(
        tofu_topic_, 10, std::bind(&ResourceUser::tofu_response_cb, this, _1));

    discharge_macaroon_sub_ = this->create_subscription<macaroon_msgs::msg::DischargeMacaroon>(
        authentication_topic_, 10, std::bind(&ResourceUser::discharge_macaroon_cb, this, _1));

    resource_macaroon_sub_ = this->create_subscription<macaroon_msgs::msg::ResourceMacaroon>(
        resource_topic_, 10, std::bind(&ResourceUser::resource_macaroon_cb, this, _1));
}

void
ResourceUser::run(void)
{
    /* might do something here. called by exec during a spin(). */
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

/*
Send a request on the tofu topic to obtain a key to create discharge macaroons

In a two party scenario, this is user->owner
In a three part scenario, this is third-party->owner
*/
void
ResourceUser::initiate_tofu(void)
{
    // publish the request
    auto msg = std::make_unique<macaroon_msgs::msg::TofuRequest>();

    msg->tofu_request = resource_name_;
    RCLCPP_INFO(this->get_logger(), "Initiating TOFU (request: %s)",
        msg->tofu_request.c_str());

    tofu_request_pub_->publish(std::move(msg));
}

/*
Send a request on the authentication topic for a given resource
*/
void
ResourceUser::initiate_authentication(void)
{
    // publish the request
    auto msg = std::make_unique<macaroon_msgs::msg::AuthenticationRequest>();

    msg->principal_name = resource_name_ + "_user";
    msg->resource_name = resource_name_;
    RCLCPP_INFO(this->get_logger(), "Initiating authentication (principal name: %s // resource name: %s)",
        msg->principal_name.c_str(), msg->resource_name.c_str());

    authentication_request_pub_->publish(std::move(msg));
}

/*
Send a request on the resource topic for the token of a given resource
*/
void
ResourceUser::request_resource_token(void)
{
    // publish the request
    auto msg = std::make_unique<macaroon_msgs::msg::ResourceTokenRequest>();

    msg->resource_name = resource_name_;
    RCLCPP_INFO(this->get_logger(), "Requesting resource token (resource: %s)",
        msg->resource_name.c_str());

    resource_token_request_pub_->publish(std::move(msg));
}

/*
Caveat the resource macaroon with a command, bind the discharge macaroon
serialise them both and send them to the resource owner
*/
void
ResourceUser::transmit_command(const std::string & command)
{
    // create a temporary macaroon and add the command as a first party caveat
    macaroons::Macaroon command_macaroon = resource_macaroon_;
    command_macaroon = command_macaroon.add_first_party_caveat(command);

    // bind the discharge macaroon to the command macaroon
    macaroons::Macaroon bound_discharge_macaroon = command_macaroon.prepare_for_request(discharge_macaroon_);

    // create a message with teh command and discharge macaroons
    auto msg = std::make_unique<macaroon_msgs::msg::CommandMacaroon>();
    msg->command_macaroon.macaroon = command_macaroon.serialize();
    msg->discharge_macaroon.macaroon = bound_discharge_macaroon.serialize();

    // publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing command: %s", command.c_str());
    command_pub_->publish(std::move(msg));
}

/*
Callback for receiving a tofu response message

Store the received key and create a discharge macaroon
*/
void
ResourceUser::tofu_response_cb(const macaroon_msgs::msg::TofuResponse::SharedPtr msg)
{
    if(msg->key.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Received a TOFU response (key: %s)", msg->key.c_str());

        // Retrieve serialized resource macaroon from the message
        discharge_macaroon_key_ = msg->key;

        // initialise and store the discharge macaroon
        initialise_discharge_macaroon();
    } else {
        RCLCPP_INFO(this->get_logger(), "TOFU response failed");
    }

}

/*
Callback for receiving a resource macaroon message

Deserialise and store the resource macaroon
*/
void
ResourceUser::resource_macaroon_cb(const macaroon_msgs::msg::ResourceMacaroon::SharedPtr msg)
{
    if(msg->resource_macaroon.macaroon.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Received resource macaroon");

        // Retrieve serialized resource macaroon from the message
        resource_macaroon_ = macaroons::Macaroon::deserialize(msg->resource_macaroon.macaroon);
    } else {
        RCLCPP_INFO(this->get_logger(), "Failed to receive a resource macaroon");
    }
}

/*
Callback for receiving a discharge macaroon message

Deserialise and store the discharge macaroon
*/
void
ResourceUser::discharge_macaroon_cb(const macaroon_msgs::msg::DischargeMacaroon::SharedPtr msg)
{
    if(msg->discharge_macaroon.macaroon.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Received discharge macaroon");

        // Retrieve serialized resource macaroon from the message
        discharge_macaroon_ = macaroons::Macaroon::deserialize(msg->discharge_macaroon.macaroon);
    } else {
        RCLCPP_INFO(this->get_logger(), "Failed to receive a discharge macaroon");
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