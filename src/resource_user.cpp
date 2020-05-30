#include "resource_user.hpp"

ResourceUser::ResourceUser(const std::string & authentication_topic, const std::string & command_topic,
                           const std::string & resource)
: ResourceBase(authentication_topic, command_topic, resource + "_user")
{
    resource_ = resource;
    ResourceBase::timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ResourceUser::run, this));

    // initialise publisher for sending authentication and resource requests
    authentication_pub_ = this->create_publisher<macaroon_msgs::msg::ResourceRequest>(authentication_topic, 10);

    // initialise publisher for sending commands to the resource owner
    command_pub_ = this->create_publisher<macaroon_msgs::msg::MacaroonCommand>(command_topic_, 10);    
}

// Send a request on the authentication topic for a given resource
void
ResourceUser::publish_authentication_request(void)
{
    // publish the request
    auto msg = std::make_unique<macaroon_msgs::msg::ResourceRequest>();
    msg->resource = ResourceBase::resource_;
    RCLCPP_INFO(this->get_logger(), "Publishing request (resource: %s)", msg->resource.c_str());

    authentication_pub_->publish(std::move(msg));
}

// Send a command macaroon and bound discharge macaroon on the command topic
void
ResourceUser::publish_command(const std::string & command)
{
    // create a temporary macaroon and add the command as a first party caveat
    macaroons::Macaroon command_macaroon = resource_macaroon;
    command_macaroon = command_macaroon.add_first_party_caveat(command);

    // bind the discharge macaroon to the command macaroon
    macaroons::Macaroon bound_discharge_macaroon = command_macaroon.prepare_for_request(discharge_macaroon);

    // create a message with teh command and discharge macaroons
    auto msg = std::make_unique<macaroon_msgs::msg::MacaroonCommand>();
    msg->command_macaroon.macaroon = command_macaroon.serialize();
    msg->discharge_macaroon.macaroon = bound_discharge_macaroon.serialize();

    // publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing command: %s", command.c_str());
    command_pub_->publish(std::move(msg));
}