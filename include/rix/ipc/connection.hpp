#pragma once

#include "rix/ipc/socket.hpp"

namespace rix {
namespace ipc {

class Server;

class Connection {
    friend class Server;

   public:
    typedef std::function<void(int status, size_t size)> IOHandler;

    Connection();
    Connection(const Connection &other);
    Connection &operator=(const Connection &other);
    ~Connection();

    int send(const Buffer &buffer);
    void async_send(const Buffer &buffer, const IOHandler &handler);
    int recv(Buffer &buffer);
    void async_recv(Buffer &buffer, const IOHandler &handler);

    Endpoint get_remote_endpoint();

   private:
    Socket socket;
    Connection(const Socket &socket);
};

}  // namespace ipc
}  // namespace rix