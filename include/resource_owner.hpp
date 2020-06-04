#ifndef RESOURCE_OWNER_HPP
#define RESOURCE_OWNER_HPP

#include "resource_user.hpp"

// Create a Resource class that subclasses the generic rclcpp::Node base class.
// The Resource class encapsulates a publisher, subscriber, Macaroon[, and CHERI token]
class ResourceOwner : public ResourceUser
{
public:
    ResourceOwner(const std::string & tofu_topic, const std::string & authentication_topic,
    const std::string & get_resource_topic, const std::string & use_resource_topic, const std::string & resource_name,
    const std::string & node_name, const bool two_party);

    // public macaroon interactions
    void add_first_party_caveat(const std::string & first_party_caveat);
    void add_valid_command_verifier(const std::string & command);

private:
    // true if this is a two-party interaction and the ResourceOwner is performing TOFU directly with the ResourceUser
    bool two_party_;

    // initialisation
    void initialise_servers(void);

    // private macaroon interactions
    void initialise_resource_macaroon(void);
    void initialise_verifier(void);
    void add_first_party_caveat_verifier(const std::string & first_party_caveat);
    bool verify_macaroon(macaroons::Macaroon resource_macaroon, std::vector<macaroons::Macaroon> discharge_macaroons);
    std::string extract_macaroon_command(macaroons::Macaroon command_macaroon);

    // only the resource owner can create a verifier (because only the owner has the key)
    macaroons::Verifier V_;

    // services
    void tofu_service(const std::shared_ptr<macaroon_msgs::srv::InitiateTofu::Request> request,
        std::shared_ptr<macaroon_msgs::srv::InitiateTofu::Response> response);
    void use_resource_token_service(const std::shared_ptr<macaroon_msgs::srv::UseResourceToken::Request> request,
        std::shared_ptr<macaroon_msgs::srv::UseResourceToken::Response> response);
};

#endif // RESOURCE_OWNER_HPP