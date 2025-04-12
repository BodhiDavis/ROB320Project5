#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include <vector>

namespace rix {
namespace ipc {

typedef std::vector<uint8_t> Buffer;

namespace SocketFlags {

typedef struct linger Linger;
typedef struct timeval Timeval;
typedef struct ip_mreq MulticastRequest;

namespace Domain {
enum { UNIX = AF_UNIX, INET = AF_INET, INET6 = AF_INET6 };
}  // namespace Domain

namespace Type {
enum { STREAM = SOCK_STREAM, DGRAM = SOCK_DGRAM, SEQPACKET = SOCK_SEQPACKET, RAW = SOCK_RAW, RDM = SOCK_RDM };
}

namespace Options {
enum {
    // Socket Options
    DEBUG = SO_DEBUG,          ///< enables recording of debugging information
    REUSEADDR = SO_REUSEADDR,  ///< enables local address reuse
    REUSEPORT = SO_REUSEPORT,  ///< enables duplicate address and port bindings
    KEEPALIVE = SO_KEEPALIVE,  ///< enables keep connections alive
    DONTROUTE = SO_DONTROUTE,  ///< enables routing bypass for outgoing messages
    LINGER = SO_LINGER,        ///< linger on close if data present
    BROADCAST = SO_BROADCAST,  ///< enables permission to transmit broadcast messages
    OOBINLINE = SO_OOBINLINE,  ///< enables reception of out-of-band data in band
    SNDBUF = SO_SNDBUF,        ///< set buffer size for output
    RCVBUF = SO_RCVBUF,        ///< set buffer size for input
    SNDLOWAT = SO_SNDLOWAT,    ///< set minimum count for output
    RCVLOWAT = SO_RCVLOWAT,    ///< set minimum count for input
    SNDTIMEO = SO_SNDTIMEO,    ///< set timeout value for output
    RCVTIMEO = SO_RCVTIMEO,    ///< set timeout value for input
    TYPE = SO_TYPE,            ///< get the type of the socket (get only)
    ERROR = SO_ERROR,          ///< get and clear error on the socket (get only)
#ifdef SO_NOSIGPIPE
    NOSIGPIPE = SO_NOSIGPIPE,  ///< do not generate SIGPIPE, instead return EPIPE
#endif
#ifdef SO_NREAD
    NREAD = SO_NREAD,  ///< number of bytes to be read (get only)
#endif
#ifdef SO_NWRITE
    NWRITE = SO_NWRITE,  ///< number of bytes written not yet sent by the protocol (get only)
#endif
#ifdef SO_LINGER_SEC
    LINGER_SEC = SO_LINGER_SEC,  ///< linger on close if data present with timeout in seconds
#endif
#ifdef SO_DOMAIN
    DOMAIN = SO_DOMAIN,  ///< get the domain of the socket (get only)
#endif
    TOS = IP_TOS,
    TTL = IP_TTL,
#ifdef IP_RECVDSTADDR
    RECVDSTADDR = IP_RECVDSTADDR,
#endif
    MULTICAST_TTL = IP_MULTICAST_TTL,
    MULTICAST_IF = IP_MULTICAST_IF,
    MULTICAST_LOOP = IP_MULTICAST_LOOP,
    ADD_MEMBERSHIP = IP_ADD_MEMBERSHIP,
    DROP_MEMBERSHIP = IP_DROP_MEMBERSHIP,
};
}  // namespace Options

namespace IO {
enum { OOB = MSG_OOB, PEEK = MSG_PEEK, WAITALL = MSG_WAITALL, DONTROUTE = MSG_DONTROUTE };
}  // namespace IO

namespace Level {
enum {
    SOCKET = SOL_SOCKET,  ///< Socket
    IP = IPPROTO_IP,      ///< Internet protocol.
    IPV6 = IPPROTO_IPV6,  ///< Internet Protocol Version 6.
    ICMP = IPPROTO_ICMP,  ///< Control message protocol.
    RAW = IPPROTO_RAW,    ///< Raw IP Packets Protocol.
    TCP = IPPROTO_TCP,    ///< Transmission control protocol.
    UDP = IPPROTO_UDP,    ///< User datagram protocol.
};
}  // namespace Level

}  // namespace SocketFlags

namespace FileFlags {

namespace AccessMode {
/**
 * @brief Enumeration of file modes. These flags specify the permissions to
 * be placed on a file when it is opened.
 *
 */
enum { RDONLY = O_RDONLY, WRONLY = O_WRONLY, RDWR = O_RDWR };
}  // namespace AccessMode

namespace Creation {
/**
 * @brief Enumeration of the file creation flags. These flags control
 * various aspects of the behavior of the open() call itself. These flags
 * cannot be retrieved or changed.
 *
 */
enum {
    CLOEXEC = O_CLOEXEC,
    CREAT = O_CREAT,
    DIRECTORY = O_DIRECTORY,
    EXCL = O_EXCL,
    NOCTTY = O_NOCTTY,
    NOFOLLOW = O_NOFOLLOW,
#ifdef O_TMPFILE
    TMPFILE = O_TMPFILE,
#endif
    TRUNC = O_TRUNC,
};
}  // namespace Creation

namespace Status {
/**
 * @brief Enumeration of the open file status flags. These effect the semantics
 * of subsequent I/O system calls and be retrieved and modified using the
 * fctnl() F_GETFL and F_SETFL operations.
 *
 */
enum {
    APPEND = O_APPEND,
    ASYNC = O_ASYNC,
#ifdef O_DIRECT
    DIRECT = O_DIRECT,
#endif
#ifdef O_NOATIME
    NOATIME = O_NOATIME,
#endif
    NONBLOCK = O_NONBLOCK,
    NODELAY = O_NDELAY,
#ifdef O_LARGEFILE
    LARGEFILE = O_LARGEFILE,
#endif
#ifdef O_PATH
    PATH = O_PATH,
#endif
    DSYNC = O_DSYNC,
    SYNC = O_SYNC,
};
}  // namespace Status

namespace Operation {
/**
 * @brief Enumeration of operation flags for fcntl.
 *
 */
enum {
    DUPFD = F_DUPFD,
    DUPFD_CLOEXEC = F_DUPFD_CLOEXEC,
    GETFD = F_GETFD,
    SETFD = F_SETFD,
    GETFL = F_GETFL,
    SETFL = F_SETFL,
    GETOWN = F_GETOWN,
    SETOWN = F_SETOWN,
#ifdef F_GETSIG
    GETSIG = F_GETSIG,
#endif
#ifdef F_SETSIG
    SETSIG = F_SETSIG
#endif
};
}  // namespace Operation

namespace Poll {
/**
 * @brief Enumeration of poll flags.
 *
 */
enum {
    IN = POLLIN,    ///< There is data to read
    PRI = POLLPRI,  ///< Writing is now possible though a write larger than the available space in a socket or pipe will
                    ///< still block (unless O_NONBLOCK is set).
    OUT = POLLOUT,  ///< There is some exceptional condition on the file descriptor
    ERR = POLLERR,  ///< Error condition (only returned in revents; ignored in events).
    HUP = POLLHUP,  ///< Hang up (only returned in revents; ignored in events).
#ifdef POLLRDHUP
    RDHUP = POLLRDHUP,  ///< Stream socket peer closed connection, or shut down writing half of connection.
#endif
    NVAL = POLLNVAL  ///< Invalid request: fd not open (only returned in revents; ignored in events).
};
}  // namespace Poll
}  // namespace FileFlags

namespace MappingFlags {

namespace Protection {
enum {
    NONE = PROT_NONE,    ///< Pages may not be accessed.
    READ = PROT_READ,    ///< Pages may be read.
    WRITE = PROT_WRITE,  ///< Pages may be written.
    EXEC = PROT_EXEC,    ///< Pages may be executed.
};
}  // namespace Protection

namespace Creation {
enum {
    ANONYMOUS = MAP_ANONYMOUS,
    FIXED = MAP_FIXED,
#ifdef MAP_LOCKED
    LOCKED = MAP_LOCKED,
#endif
#ifdef MAP_HUGETLB
    HUGETLB = MAP_HUGETLB,
#endif
    NORESERVE = MAP_NORESERVE,
    PRIVATE = MAP_PRIVATE,
#ifdef MAP_POPULATE
    POPULATE = MAP_POPULATE,
#endif
    SHARED = MAP_SHARED,
#ifdef MAP_UNINITIALIZED
    UNINITIALIZED = MAP_UNINITIALIZED
#endif
};
}  // namespace Creation

namespace Sync {
enum {
    ASYNC = MS_ASYNC,           ///< Return immediately
    SYNC = MS_SYNC,             ///< Perform synchronous writes
    INVALIDATE = MS_INVALIDATE  ///< Invalidate all cached data
};
}  // namespace Sync

}  // namespace MappingFlags

namespace SharedObjectFlags {

namespace Creation {
enum {
    RDONLY = O_RDONLY,  ///< open for reading only
    RDWR = O_RDWR,      ///< open for reading and writing
    CREAT = O_CREAT,    ///< create object if it does not exist
    EXCL = O_EXCL,      ///< error if create and object exists
    TRUNC = O_TRUNC     ///< truncate size to 0
};

}  // namespace Creation
}  // namespace SharedObjectFlags

namespace SemaphoreFlags {

namespace Creation {
enum {
    CREAT = O_CREAT,  ///< create the semaphore if it does not exist
    EXCL = O_EXCL,    ///< error if create and semaphore exists
};

}  // namespace Creation
}  // namespace SemaphoreFlags

}  // namespace ipc
}  // namespace rix