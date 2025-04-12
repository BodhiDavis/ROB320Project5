#pragma once

#include <errno.h>

namespace rix {
namespace ipc {

enum Errno {
    PERM = -EPERM,
    NOENT = -ENOENT,
    SRCH = -ESRCH,
    INTR = -EINTR,
    IO = -EIO,
    NXIO = -ENXIO,
    TOOBIG = -E2BIG,
    NOEXEC = -ENOEXEC,
    BADF = -EBADF,
    CHILD = -ECHILD,
    AGAIN = -EAGAIN,
    NOMEM = -ENOMEM,
    ACCES = -EACCES,
    FAULT = -EFAULT,
    NOTBLK = -ENOTBLK,
    BUSY = -EBUSY,
    EXIST = -EEXIST,
    XDEV = -EXDEV,
    NODEV = -ENODEV,
    NOTDIR = -ENOTDIR,
    ISDIR = -EISDIR,
    INVAL = -EINVAL,
    NFILE = -ENFILE,
    MFILE = -EMFILE,
    NOTTY = -ENOTTY,
    TXTBSY = -ETXTBSY,
    FBIG = -EFBIG,
    NOSPC = -ENOSPC,
    SPIPE = -ESPIPE,
    ROFS = -EROFS,
    MLINK = -EMLINK,
    PIPE = -EPIPE,
    DOM = -EDOM,
    RANGE = -ERANGE
};

}  // namespace ipc
}  // namespace rix