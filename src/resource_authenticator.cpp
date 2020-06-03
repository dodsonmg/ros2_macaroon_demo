#include "resource_authenticator.hpp"

ResourceAuthenticator::ResourceAuthenticator(const std::string & tofu_topic, const std::string & authentication_topic,
    const std::string & get_resource_topic, const std::string & use_resource_topic, const std::string & resource_name,
    const std::string & node_name)
: ResourceUser(tofu_topic, authentication_topic, get_resource_topic, use_resource_topic, resource_name, node_name)
{
    initialise_servers();
    initialise_clients();
}

void
ResourceAuthenticator::initialise_servers(void)
{
    authentication_server_ = this->create_service<macaroon_msgs::srv::Authenticate>(authentication_topic_,
        std::bind(&ResourceAuthenticator::authentication_service, this, _1, _2));

    get_resource_token_server_ = this->create_service<macaroon_msgs::srv::GetResourceToken>(get_resource_topic_,
        std::bind(&ResourceAuthenticator::get_resource_token_service, this, _1, _2));
}

void
ResourceAuthenticator::initialise_clients(void)
{
    tofu_client_ = this->create_client<macaroon_msgs::srv::InitiateTofu>(tofu_topic_);
}

/**
 * Authentication service
 *
 * Receives an authentication request.
 * Responds with either a serialised discharge macaroon or rejection (empty string)
 */
void
ResourceAuthenticator::authentication_service(const std::shared_ptr<macaroon_msgs::srv::Authenticate::Request> request,
    std::shared_ptr<macaroon_msgs::srv::Authenticate::Response> response)
{
    // if the resource and principal are valid, respond with a serialised discharge macaroon
    if(request->resource_name != resource_name_) {
        response->discharge_macaroon.macaroon = "";

        RCLCPP_INFO(this->get_logger(), "Authentication:\tInvalid request");
        RCLCPP_INFO(this->get_logger(), "> resource:\t%s is unknown", request->resource_name.c_str());
    } else if(request->principal_name != resource_name_ + "_user") {
        response->discharge_macaroon.macaroon = "";

        RCLCPP_INFO(this->get_logger(), "Authentication:\tInvalid request");
        RCLCPP_INFO(this->get_logger(), "> principal:\t%s is unknown", request->principal_name.c_str());
    } else {
        // serialise the discharge macaroon and add it to the message
        response->discharge_macaroon.macaroon = discharge_macaroon_.serialize();

        RCLCPP_INFO(this->get_logger(), "Authentication:\tValid request");
        RCLCPP_INFO(this->get_logger(), "> principal:\t%s", request->principal_name.c_str());
        RCLCPP_INFO(this->get_logger(), "> resource:\t%s", request->resource_name.c_str());
        RCLCPP_INFO(this->get_logger(), "> discharge macaroon id:\t\t%s", discharge_macaroon_.identifier().c_str());
        RCLCPP_INFO(this->get_logger(), "> discharge macaroon signature:\t%s", discharge_macaroon_.signature_string().c_str());
    }
}

/**
 * GetResourceToken service
 *
 * Receives an resource request.
 * Responds with either a serialised resource macaroon or rejection (empty string)
 */
void
ResourceAuthenticator::get_resource_token_service(const std::shared_ptr<macaroon_msgs::srv::GetResourceToken::Request> request,
    std::shared_ptr<macaroon_msgs::srv::GetResourceToken::Response> response)
{
    // if the resource and principal are valid, respond with a serialised discharge macaroon
    if(request->resource_name != resource_name_) {
        response->resource_macaroon.macaroon = "";
        RCLCPP_INFO(this->get_logger(), "Token request:\tInvalid request");
        RCLCPP_INFO(this->get_logger(), "> resource:\t%s is unknown.", request->resource_name.c_str());
    } else {
        // serialise the discharge macaroon and add it to the message
        response->resource_macaroon.macaroon = resource_macaroon_.serialize();

        RCLCPP_INFO(this->get_logger(), "Token request:\tValid request");
        RCLCPP_INFO(this->get_logger(), "> resource:\t%s", request->resource_name.c_str());
        RCLCPP_INFO(this->get_logger(), "> resource macaroon id:\t\t%s", resource_macaroon_.identifier().c_str());
        RCLCPP_INFO(this->get_logger(), "> resource macaroon signature:\t%s", resource_macaroon_.signature_string().c_str());
    }
}