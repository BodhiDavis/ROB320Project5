#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "rix/ipc/flags.hpp"
#include "rix/ipc/endpoint.hpp"
#include "rix/ipc/file.hpp"

namespace rix {
namespace ipc {

class Socket : public File {
   public:
    typedef std::function<void(int status)> AsyncAcceptHandler;

    /**
     * @brief Construct a new Socket object. Sets fd to -1 by default.
     * 
     */
    Socket();

    /**
     * @brief Construct a new Socket object. This will call `socket` to create
     * a new socket with the specified domain and type.
     * 
     * @param domain The domain of the socket.
     * @param type The type of the socket.
     */
    Socket(int domain, int type);

    /**
     * @brief Bind the socket to the specified endpoint.
     * 
     * @param endpoint The endpoint to bind to.
     * @return -1 on failure, 0 on success
     */
    int bind(const Endpoint &endpoint);

    /**
     * @brief Enable a stream socket to accept incoming connections from other
     * sockets.
     * 
     * @param backlog The maximum length to which the queue of pending connections
     * for the socket may grow.
     * @return -1 on failure, 0 on success
     */
    int listen(int backlog);

    /**
     * @brief Establish a connection with another socket.
     * 
     * @param endpoint The endpoint to connect to.
     * @return -1 on failure, 0 on success
     */
    int connect(const Endpoint &endpoint);

    /**
     * @brief Start an asynchronous connect operation. This operation will call
     * `connect` in a background thread. Once `connect` returns, a task will be 
     * enqueued to the Executor that will invoke the handler with a status code
     * (-1 for failure, 0 for success).
     * 
     * @param endpoint The endpoint to connect to.
     * @param handler The callback function to be invoked once the connect
     * operation has completed.
     * @return -1 on failure, 0 on success
     */
    int async_connect(const Endpoint &endpoint, const AsyncHandler &handler);

    /**
     * @brief Accept a connection from a peer application on a listening stream
     * socket.
     * 
     * @param sock The new socket created by accepting a connection. If 
     * successful, `is_open` will return true.
     * @return -1 on failure, the file descriptor of the accepted socket on 
     * success.
     */
    int accept(Socket &sock);

    /**
     * @brief Start an asynchronous accept operation. This operation will make a
     * poll system call in a background thread with the POLLIN flag set in 
     * events. Once poll returns, a task will be enqueued to the Executor that 
     * will accept the connection and invoke the handler with a status code (-1
     * for failure, 0 for success) and the file descriptor of the accepted
     * socket.
     * 
     * @param sock The new socket created by accepting a connection, If
     * successful, `is_open` will return true.
     * @param handler The callback function to be invoked once the accept
     * operation has completed.
     * @return -1 on failure, 0 on success
     */
    int async_accept(Socket &sock, const AsyncAcceptHandler &handler);

    /**
     * @brief Read data from the socket. This call is equivalent to `read`
     * with `flags` set to 0.
     * 
     * @param buffer The buffer to store the read data.
     * @param size The number of bytes to read.
     * @param flags The flags to use.
     * @return -1 on failure, the number of bytes read on success.
     */
    int recv(Buffer &buffer, size_t size, int flags);

    /**
     * @brief Start an asynchronous recv operation. This operation will make a
     * poll system call in a background thread with the POLLIN flag set in
     * events. Once poll returns, a task will be enqueued to the Executor that
     * will recv from the socket and invoke the handler with a status code (-1
     * for failure, 0 for success) and the number of bytes that were read.
     * 
     * @param buffer The buffer to store the read data.
     * @param size The number of bytes to read.
     * @param flags The recv flags to use.
     * @param handler The callback function to be invoked once the recv
     * operation has completed.
     * @return -1 on failure, 0 on success
     */
    int async_recv(Buffer &buffer, size_t size, int flags, const AsyncIOHandler &handler);

    /**
     * @brief Write data to the socket. This call is equivalent to `write`
     * with `flags` set to 0.
     * 
     * @param buffer The data to be written.
     * @param size The number of bytes to send.
     * @param flags The send flags to use.
     * @return -1 on failure, the number of bytes sent on success.
     */
    int send(const Buffer &buffer, size_t size, int flags);

    /**
     * @brief Start an asynchronous send operation. This operation will make a
     * poll system call in a background thread with the POLLOUT flag set in
     * events. Once poll returns, a task will be enqueued to the Executor that
     * will send to the socket and invoke the handler with a status code (-1
     * for failure, 0 for success) and the number of bytes that were sent.
     * 
     * @param buffer The data to be written.
     * @param size The number of bytes to send.
     * @param flags The send flags to use.
     * @param handler The callback function to be invoked once the send
     * operation has completed.
     * @return -1 on failure, 0 on success
     */
    int async_send(const Buffer &buffer, size_t size, int flags, const AsyncIOHandler &handler);
    
    /**
     * @brief Read data from a datagram socket. The endpoint parameter will be
     * filled with the address of the sending socket.
     * 
     * @param buffer The buffer to store the read data.
     * @param size The number of bytes to read.
     * @param flags The recv flags to use.
     * @param endpoint The endpoint of the peer socket.
     * @return -1 on failure, the number of bytes read on success.
     */
    int recvfrom(Buffer &buffer, size_t size, int flags, Endpoint &endpoint);

    /**
     * @brief Start an asynchronous recvfrom operation. This operation will make
     * a poll system call in a background thread with the POLLIN flag set in
     * events. Once poll returns, a task will be enqueued to the Executor that
     * will recvfrom the socket, fill the endpoint parameter, and invoke the 
     * handler with a status code (-1 for failure, 0 for success) and the number
     * of bytes that were read.
     * 
     * @param buffer The buffer to store the read data.
     * @param size The number of bytes to read.
     * @param flags The recv flags to use.
     * @param endpoint The endpoint of the peer socket.
     * @param handler The callback function to be invoked once the recvfrom
     * operation has completed.
     * @return -1 on failure, 0 on success
     */
    int async_recvfrom(Buffer &buffer, size_t size, int flags, Endpoint &endpoint, const AsyncIOHandler &handler);

    /**
     * @brief Write data to a datagram socket. The endpoint parameter specifies
     * the destination address.
     * 
     * @param buffer The data to be written.
     * @param size The number of bytes to send.
     * @param flags The send flags to use.
     * @param endpoint The endpoint of the peer socket.
     * @return -1 on failure, the number of bytes sent on success.
     */
    int sendto(const Buffer &buffer, size_t size, int flags, const Endpoint &endpoint);

    /**
     * @brief Start an asynchronous sendto operation. This operation will make
     * a poll system call in a background thread with the POLLOUT flag set in
     * events. Once poll returns, a task will be enqueued to the Executor that
     * will sendto the socket and invoke the handler with a status code (-1 for 
     * failure, 0 for success) and the number of bytes that were sent.
     * 
     * @param buffer The data to be written.
     * @param size The number of bytes to send.
     * @param flags The send flags to use.
     * @param endpoint The endpoint of the peer socket.
     * @param handler The callback function to be invoked once the sendto
     * operation has completed.
     * @return -1 on failure, 0 on success
     */
    int async_sendto(const Buffer &buffer, size_t size, int flags, const Endpoint &endpoint, const AsyncIOHandler &handler);

    /**
     * @brief Get the value of a socket option.
     * 
     * @tparam T The type of the option value. This will always be `int` 
     * unless the option is SO_LINGER (which uses `Socket::Linger`) or 
     * SO_RCVTIMEO or SO_SNDTIMEO (which use `Socket::Timeval`).
     * @param level The level at which the option is defined.
     * @param name The name of a specified socket option.
     * @param value The option data.
     * @return -1 on failure, 0 on success.
     */
    template<typename T>
    int getsockopt(int level, int name, T &value);

    /**
     * @brief Set the value of a socket option.
     * 
     * @tparam T The type of the option value. This will always be `int` 
     * unless the option is SO_LINGER (which uses `Socket::Linger`) or 
     * SO_RCVTIMEO or SO_SNDTIMEO (which use `Socket::Timeval`).
     * @param level The level for which the option is being set.
     * @param name The name of a specified socket option.
     * @param value The option data.
     * @return -1 on failure, 0 on success.
     */
    template<typename T>
    int setsockopt(int level, int name, const T &value);

    /**
     * @brief Get the local address of the socket.
     * 
     * @param endpoint The local endpoint.
     * @return -1 on failure, 0 on success.
     */
    int getsockname(Endpoint &endpoint) const;

    /**
     * @brief Get the remote address of the socket.
     * 
     * @param endpoint The remote endpoint.
     * @return -1 on failure, 0 on success.
     */
    int getpeername(Endpoint &endpoint) const;

    bool is_bound() const;
    bool is_listening() const;

   private:
    int domain;
    int type;
    bool _bound;
    bool _listening;

    using File::open;
    using File::async_open;

    Socket(int fd, int domain, int type);

    void connect_async_operation(const Endpoint &endpoint, const AsyncHandler &handler);

    void accept_async_operation(Socket &sock, const AsyncAcceptHandler &handler);
    void accept_completion_handler(Socket &sock, const AsyncAcceptHandler &handler);

    void recv_async_operation(Buffer &buffer, size_t size, int flags, const AsyncIOHandler &handler);
    void recv_completion_handler(Buffer &buffer, size_t size, int flags, const AsyncIOHandler &handler);

    void send_async_operation(const Buffer &buffer, size_t size, int flags, const AsyncIOHandler &handler);
    void send_completion_handler(const Buffer &buffer, size_t size, int flags,const AsyncIOHandler &handler);

    void recvfrom_async_operation(Buffer &buffer, size_t size, int flags, Endpoint &endpoint, const AsyncIOHandler &handler);
    void recvfrom_completion_handler(Buffer &buffer, size_t size, int flags, Endpoint &endpoint, const AsyncIOHandler &handler);

    void sendto_async_operation(const Buffer &buffer, size_t size, int flags, const Endpoint &endpoint, const AsyncIOHandler &handler);
    void sendto_completion_handler(const Buffer &buffer, size_t size, int flags, const Endpoint &endpoint, const AsyncIOHandler &handler);
};

template<typename T>
int Socket::getsockopt(int level, int optname, T &value) {
    socklen_t len = sizeof(T);
    return ::getsockopt(fd, level, optname, &value, &len);
}

template<typename T>
int Socket::setsockopt(int level, int optname, const T &value) {
    return ::setsockopt(fd, level, optname, &value, sizeof(T));
}

}  // namespace ipc
}  // namespace rix
