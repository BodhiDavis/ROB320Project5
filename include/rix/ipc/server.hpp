#pragma once

#include <atomic>
#include <unordered_set>

#include "rix/ipc/connection.hpp"
#include "rix/ipc/socket.hpp"

namespace rix {
namespace ipc {

class Server {
   public:
    typedef std::function<void(int status)> AcceptHandler;
    typedef std::function<void(int status)> CloseHandler;
    typedef std::function<void(int status, std::weak_ptr<Connection> conn)> SendAllHandler;

    Server(const Endpoint &endpoint);
    Server();
    Server(const Server &other);
    Server &operator=(const Server &other);
    ~Server();

    int listen(int backlog);

    int accept(std::weak_ptr<Connection> &connection);
    void async_accept(std::weak_ptr<Connection> &connection, const AcceptHandler &handler);

    void close(const std::weak_ptr<Connection> &connection);
    void close();

    bool is_bound() const;
    bool is_listening() const;

    Endpoint get_local_endpoint() const;

   private:
    Socket socket;
    Socket accepted;
    std::unordered_set<std::shared_ptr<Connection>> connections;

    void accept_completion_handler(std::weak_ptr<Connection> &connection, const AcceptHandler &handler, int status);
};

}  // namespace ipc
}  // namespace rix