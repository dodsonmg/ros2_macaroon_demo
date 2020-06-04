# ROS2 Macaroons Demo

This repository contains a proof of concept for integrating Macaroon tokens [0] with ROS2 (2nd gen Robot Operating System).

A Macaroon is used as a token for accessing a resource.  Providing a macaroon as part of a request to access some resource is a necessary, though possibly not sufficient, proof that the bearer has the right to access the resource.  Macaroons can be attenuated to support delegating reduced privileges.  Macaroons also support third-party discharge requirements, whereby the bearer of the resource macaroon has to authenticate with a third-party, obtain a discharge macaroon from that third party, and then present the resource and discharge macaroons together as proof that the bearer can access the resource.

In this demo, the resource is the ability to change the velocity of a node (loosely approximating the ability to send a `cmd_vel` signal to a TurtleBot).

The demo attempts to answer several questions, such as:
1. How is the Macaroon provided from the resource owner to the user?
2. How does the user authenticate itself with the owner?
3. How can a token be delegated?

To avoid establishing a PKI and having to perform complex, asymmetric cryptographic operations, we assume there is at least one Trust on First Use (TOFU) operation (e.g., based on physical contact between two devices) [1].  This is considered an initial authorisation step and material can be "securely" passed from the ResourceOwner to some other party (either the ResourceUser directly or the ResourceAuthenticator).

We have both a two and three party scenario.  The two-party scenario involves a ResourceOwner and ResourceUser only.  The three-party scenario involves a ResourceAuthenticator that, like a trusted third party, authenticates a ResourceUser before they can make use of a resource macaroon.  In both cases, the ResourceUser can delegate its resource macaroon, possibly attenuating it to reduce the level of access.  In the three-party scenario, any ResourceUser to whom a resource macaroon has been delegated will also have to authenticate with the ResourceAuthenticator.

In a two-party scenario:
- ResourceUser performs TOFU with a ResourceOwner and obtains the resource macaroon and a key for a discharge macaroon
- ResourceUser provides the command token to the ResourceOwner

In a three-party scenario:
- ResourceAuthenticator performs TOFU with a ResourceOwner and obtains the resource macaroon and a key for a discharge macaroon
- ResourceUser requests a resource token from the ResourceAuthenticator (a bit contrived, but if this had been delegated multiple times, the ResourceUser would have the resource macaroon before authenticating with the third-party)
- ResourceUser performs authentication with the ResourceAuthenticator and obtains a discharge macaroon
- ResourceUser provides a command token to the ResourceOwner

[0] Birgisson, Arnar, Joe Gibbs Politz, Úlfar Erlingsson, Ankur Taly, Michael Vrable, and Mark Lentczner. ‘Macaroons: Cookies with Contextual Caveats for Decentralized Authorization in the Cloud’. Network and Distributed System Security (NDSS) Symposium, 2014. https://doi.org/10.14722/ndss.2014.23212.

[1] Stajano, Frank, and Ross Anderson. ‘The Resurrecting Duckling: Security Issues for Ad-Hoc Wireless Networks’. In Security Protocols, edited by Bruce Christianson, Bruno Crispo, James A. Malcolm, and Michael Roe, 1796:172–82. Lecture Notes in Computer Science. Berlin, Heidelberg: Springer Berlin Heidelberg, 2000. https://doi.org/10.1007/10720107_24.
