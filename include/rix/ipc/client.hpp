#pragma once

#include "rix/ipc/socket.hpp"

namespace rix {
namespace ipc {

class Client {
  public:
    typedef std::function<void(int status)> ConnectHandler;
    typedef std::function<void(int status, size_t size)> IOHandler;

    Client();
    Client(const Client &other);
    Client &operator=(const Client &other);
    ~Client();

    bool connect(const Endpoint &endpoint);
    void async_connect(const Endpoint &endpoint, const ConnectHandler &handler);
    int send(const Buffer &buffer);
    void async_send(const Buffer &buffer, const IOHandler &handler);
    int recv(Buffer &buffer);
    void async_recv(Buffer &buffer, const IOHandler &handler);
    void close();

    bool is_connected() const;

  private:
    Socket socket;
    std::atomic<bool> connected;

    void connect_completion_handler(const ConnectHandler &handler, int status);
    void recv_completion_handler(int status, size_t size, const IOHandler &handler);
    void send_completion_handler(int status, size_t size, const IOHandler &handler);
};

}  // namespace ipc
}  // namespace rix