#pragma once

#include "rix/ipc/socket.hpp"

namespace rix {
namespace ipc {

class Peer {
   public:
    typedef std::function<void(const Buffer &buffer, const Endpoint &ep)> PeerRecvFromHandler;
    typedef std::function<void(size_t size, const Endpoint &ep)> PeerSendToHandler;
    typedef std::function<void()> PeerCloseHandler;
    typedef std::function<void(int error)> PeerErrorHandler;

    Peer(const Endpoint &endpoint);
    Peer(const Endpoint &endpoint, const PeerErrorHandler &handler);
    Peer();
    Peer(const Peer &other);
    Peer &operator=(const Peer &other);
    ~Peer();

    void on_recvfrom(const PeerRecvFromHandler &handler);
    void on_sendto(const PeerSendToHandler &handler);
    void on_close(const PeerCloseHandler &handler);
    void on_error(const PeerErrorHandler &handler);

    int join_multicast_group(const std::string &address);
    int leave_multicast_group(const std::string &address);

    void set_recv_size(size_t size);
    void sendto(const Buffer &buffer, size_t size, const Endpoint &ep);
    void close();

    void run();
    void shutdown();
    bool is_running() const;

   private:
    Socket socket;
    Buffer buffer;
    Endpoint recv_ep;
    size_t recv_size;
    std::atomic<bool> running;
    PeerRecvFromHandler recvfrom_handler;
    PeerSendToHandler sendto_handler;
    PeerCloseHandler close_handler;
    PeerErrorHandler error_handler;

    void init(const Endpoint &ep);
    void recvfrom();
    void recvfrom_completion_handler(int status, size_t size);
    void sendto_completion_handler(const Endpoint &ep, int status, size_t size);
};

}  // namespace ipc
}  // namespace rix