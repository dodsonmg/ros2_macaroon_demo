#ifndef RESOURCE_AUTHENTICATOR_HPP
#define RESOURCE_AUTHENTICATOR_HPP

#include "resource_user.hpp"

// Create a Resource class that subclasses the generic rclcpp::Node base class.
// The Resource class encapsulates a publisher, subscriber, Macaroon[, and CHERI token]
class ResourceAuthenticator : public ResourceUser
{
public:
    ResourceAuthenticator(const std::string & tofu_topic, const std::string & authentication_topic,
    const std::string & get_resource_topic, const std::string & use_resource_topic, const std::string & resource_name,
    const std::string & node_name);

private:
    // initialisation
    void initialise_servers(void);
    void initialise_clients(void);

    // services (authenticator just needs to authenticate with the user and provide a resource token)
    void authentication_service(const std::shared_ptr<macaroon_msgs::srv::Authenticate::Request> request,
        std::shared_ptr<macaroon_msgs::srv::Authenticate::Response> response);
    void get_resource_token_service(const std::shared_ptr<macaroon_msgs::srv::GetResourceToken::Request> request,
        std::shared_ptr<macaroon_msgs::srv::GetResourceToken::Response> response);
};

#endif // RESOURCE_AUTHENTICATOR_HPP