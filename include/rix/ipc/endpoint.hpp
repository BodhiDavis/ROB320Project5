#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ifaddrs.h>

#include <iostream>
#include <vector>
#include <string>
#include <cstring>

namespace rix {
namespace ipc {

/**
 * @brief Class for representing Endpoints.
 * 
 * @example tcp_client.cpp
 * @example tcp_client_noblock.cpp
 * @example tcp_server.cpp
 * @example tcp_server_noblock.cpp
 * @example udp_receiver.cpp
 * @example udp_receiver_multicast.cpp
 * @example udp_sender.cpp
 * @example udp_sender_multicast.cpp
 * @example web_client.cpp
 * @example web_server.cpp
*/
class Endpoint {
  public:
    std::string address;
    int port;

    static const std::vector<std::string> &get_addresses();

    /**
     * @brief Default constructor. Initializes an empty Endpoint.
     */
    Endpoint();

    /**
     * @brief Constructs a Endpoint with the given IP address and port.
     * @param address The IP address.
     * @param port The port number.
     * @param is_ws Whether the Endpoint is a WebSocket Endpoint.
     */
    Endpoint(const std::string &address, int port);

    /**
     * @brief Less-than operator for comparing Endpoints.
     * @param other The other Endpoint to compare with.
     * @return true if this Endpoint is less than the other Endpoint, false otherwise.
     */
    bool operator<(const Endpoint &other) const;

    /**
     * @brief Equality operator for comparing Endpoints.
     * @param other The other Endpoint to compare with.
     * @return true if the Endpoints are equal, false otherwise.
     */
    bool operator==(const Endpoint &other) const;

    /**
     * @brief Inequality operator for comparing Endpoints.
     * @param other The other Endpoint to compare with.
     * @return true if the Endpoints are not equal, false otherwise.
     */
    bool operator!=(const Endpoint &other) const;

    /**
     * @brief Converts the Endpoint to a string representation.
     * @return The string representation of the Endpoint.
     */
    std::string to_string() const;

    /**
     * @brief Stream insertion operator for printing Endpoints.
     * @param os The output stream.
     * @param uri The Endpoint to print.
     * @return The output stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const Endpoint &uri) {
        os << uri.to_string();
        return os;
    }

    struct Hash {
        std::size_t operator()(const Endpoint &endpoint) const {
            return std::hash<std::string>{}(endpoint.to_string());
        }
    };
};

}  // namespace ipc
}  // namespace rix