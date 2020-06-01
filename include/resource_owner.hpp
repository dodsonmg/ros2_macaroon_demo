#ifndef RESOURCE_OWNER_HPP
#define RESOURCE_OWNER_HPP

#include "resource_user.hpp"

// Create a Resource class that subclasses the generic rclcpp::Node base class.
// The Resource class encapsulates a publisher, subscriber, Macaroon[, and CHERI token]
class ResourceOwner : public ResourceUser
{
public:
    ResourceOwner(const std::string & tofu_topic, const std::string & authentication_topic,
        const std::string & resource_topic, const std::string & resource_name, const std::string & node_name);

    // public macaroon interactions
    void add_first_party_caveat(const std::string & first_party_caveat);
    void add_valid_command_verifier(const std::string & command);

private:
    // initialisation
    void initialise_publishers(void);
    void initialise_subscribers(void);

    // private macaroon interactions
    void initialise_resource_macaroon(void);
    void initialise_verifier(void);
    void add_first_party_caveat_verifier(const std::string & first_party_caveat);
    bool verify_macaroon(macaroons::Macaroon resource_macaroon, std::vector<macaroons::Macaroon> discharge_macaroons);
    std::string extract_macaroon_command(macaroons::Macaroon command_macaroon);

    // only the resource owner can create a verifier (because only the owner has the key)
    macaroons::Verifier V_;

    // callbacks
    void tofu_request_cb(const macaroon_msgs::msg::TofuRequest::SharedPtr msg);                         // respond to tofu request with a discharge macaroon key or a rejection
    void authentication_request_cb(const macaroon_msgs::msg::AuthenticationRequest::SharedPtr msg);     // respond to an authentication request with a discharge macaroon or a rejection
    void resource_token_request_cb(const macaroon_msgs::msg::ResourceTokenRequest::SharedPtr msg);      // respond to a request for a resource token with a resource macaroon or a rejection
    void command_cb(const macaroon_msgs::msg::CommandMacaroon::SharedPtr msg);                          // respond to a command macaroon (a caveated resource macaroon and a bound discharge macaroon), deserialise them, validate them, extract the command, and act or reject
};

#endif // RESOURCE_OWNER_HPP