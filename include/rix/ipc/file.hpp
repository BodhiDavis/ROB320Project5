#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>

#include "rix/ipc/flags.hpp"
#include "rix/ipc/errno.hpp"
#include "rix/ipc/async.hpp"
#include "rix/ipc/executor.hpp"
#include "rix/ipc/endpoint.hpp"

namespace rix {
namespace ipc {
  
class File {
  public:
  /**
   * @brief Default callback type for an asynchronous operation.
   * 
   */
    typedef std::function<void(int status)> AsyncHandler;

    /**
     * @brief Callback type for an asynchronous open operation.
     * 
     */
    typedef std::function<void(int status, int fd)> AsyncOpenHandler;

    /**
     * @brief Callback type for an asynchronous read operation.
     * 
     */
    typedef std::function<void(int status, size_t size)> AsyncIOHandler;

    /**
     * @brief Open a file. 
     * 
     * @param pathname The path to the file
     * @param creation_flags Bitmask of file creation flags.
     * @param mode The mode to open the file in
     * @return -1 on failure, a file descriptor on success
     */
    static int open(std::string pathname, int creation_flags, mode_t mode);

    /**
     * @brief Start an asynchronous open operation. This operation will make an
     * open system call in a background thread. Once this operation completes,
     * a task will be enqueued to the Executor that will invoke the handler with
     * a status code (-1 for failure, 0 for success) and the file descriptor of
     * the opened file.
     * 
     * @param pathname The path to the file
     * @param creation_flags Bitmask of file creation flags.
     * @param mode The mode to open the file in
     * @param handler The callback function to be invoked once the open 
     * operation has completed
     * @return -1 on failure, 0 on success
     */
    static std::thread::id async_open(std::string pathname, int creation_flags, mode_t mode, const AsyncOpenHandler &handler);

    /**
     * @brief Construct a new File object. Sets fd to -1 by default. 
     * 
     */
    File();

    /**
     * @brief Construct a new File object.
     * 
     * @param fd The file descriptor
     */
    File(int fd);

    /**
     * @brief Copy constructor. This will invoke `dup` to create a copy of the 
     * file descriptor. If successful, `is_open` will return true.
     * 
     * @param other 
     */
    File(const File &other);

    /**
     * @brief Copy assignment operator. This will invoke `dup` to create a copy of 
     * the file descriptor. If successful, `is_open` will return true.
     * 
     * @param other 
     * @return File& 
     */
    File &operator=(const File &other);

    /**
     * @brief Destroy the File object. This will invoke `close` if the file
     * is open.
     * 
     */
    ~File();

    /**
     * @brief Retrieve or modify the access mode and file status flags of
     * an open file.
     * 
     * @param op_flag The operation flag
     * @param status_flags Optional status flags for arguments to the specified
     * operation
     * @return -1 on failure, 0 on success
     */
    int fcntl(int op_flag, int status_flags = 0) const; 

    /**
     * @brief Check if a file descriptor is ready for an I/O operation.
     * 
     * @param events Bitmask of `PollFlags`
     * @param timeout Number of milliseconds to block for (0 will return 
     * immediately, -1 will block indefinitely)
     * @return -1 on failure, 0 on success
     */
    int poll(int events, int timeout = -1) const;

    /**
     * @brief Read data from the file.
     * 
     * @param buffer The buffer to store the read data. Must be at least `size`
     * bytes long.
     * @param size The number of bytes to read.
     * @return -1 on failure, number of bytes read on success
     */
    int read(Buffer &buffer, size_t size) const;

    /**
     * @brief Start an asynchronous read operation. This operation will make a
     * poll system call in a background thread with the POLLIN flag set in 
     * events. Once poll returns, a task will be enqueued to the Executor that 
     * will read from the file and invoke the handler with a status code (-1 for
     * failure, 0 for success) and the number of bytes that were read. The task 
     * will occur in the thread calling `Executor::run` to minimize the 
     * likelihood of race conditions when writing to the buffer. 
     * 
     * @warning The lifetime of `buffer` must outlast the read call.
     * 
     * @param buffer The buffer to store the read data. Must be at least `size`
     * bytes long.
     * @param size The number of bytes to read.
     * @param handler The callback function to be invoked once the read 
     * operation has completed.
     * @return -1 on failure, 0 on success 
     */
    std::thread::id async_read(Buffer &buffer, size_t size, const AsyncIOHandler &handler);

    /**
     * @brief Write data to the file.
     * 
     * @param buffer The data to be written. Must be at least `size` bytes long.
     * @param size The number of bytes to send.
     * @return -1 on failure, 0 on success
     */
    int write(const Buffer &buffer, size_t size) const; 

    /**
     * @brief Start an asynchronous write operation. This operation will make a
     * poll system call in a background thread with the POLLOUT flag set in 
     * events. Once poll returns, a task will be enqueued to the Executor that 
     * will write to the file and invoke the handler with a status code (-1 for
     * failure, 0 for success) and the number of bytes that were sent. The task 
     * will occur in the thread calling `Executor::run` to minimize the 
     * likelihood of race conditions when writing the buffer to the file.
     * 
     * @warning The lifetime of `buffer` must outlast the read call.
     * 
     * @param buffer The buffer to write to the file. Must be at least `size`
     * bytes long.
     * @param size The number of bytes to write.
     * @param handler The callback function to be invoked once the write 
     * operation has completed.
     * @return -1 on failure, 0 on success
     */
    std::thread::id async_write(const Buffer &buffer, size_t size, const AsyncIOHandler &handler);

    /**
     * @brief Close the file.
     * 
     * @return -1 on failure, 0 on success
     */
    int close();

    /**
     * @brief Get the file descriptor.
     * 
     * @return The file descriptor
     */
    int get_fd() const;

    /**
     * @brief Get the access mode of the file. This will return the access mode
     * that appears in the file status flags.
     * 
     * @return The access mode of the file. 
     */
    int get_access_mode() const;

    /**
     * @brief Get the file status flags. Equivalent to 
     * `fcntl(OpFlags::GETFL, 0)`.
     * 
     * @return int 
     */
    int get_flags() const;

    /**
     * @return true if the file descriptor is valid
     */
    bool is_open() const;   

    /**
     * @brief Equivalent to `poll(PollFlags::IN, 0)`.
     * 
     * @return true if input is available.
     */
    bool is_readable() const;

    /**
     * @brief  Equivalent to `poll(PollFlags::OUT, 0)`.
     * 
     * @return true if output is possible.
     */
    bool is_writable() const;

    void set_nonblocking(bool status);

    bool is_nonblocking();

  protected:
    int fd;
    
  private:
    /**
     * @brief The completion handler invoked by `Executor` after the 
     * asynchronous open operation is complete. This will set the file 
     * descriptor for the `File` object and invoke the specified handler.
     * 
     * @param fd The file descriptor created by `async_open_helper`
     * @param handler The callback function to run after the asynchronous operation is complete
     */
    static void open_completion_handler(int fd, AsyncOpenHandler &handler);

    /**
     * @brief The asynchronous open operation. This will call `::open`. Once 
     * `::open` returns, it will post the completion handler 
     * `open_completion_handler` to the `Executor`.
     * 
     * @param pathname The path to the file
     * @param creation_flags Bitmask of file creation flags.
     * @param mode The mode to open the file in
     * @param handler The callback function to be invoked once the open 
     * operation has completed
     */
    static void open_async_operation(const std::string &pathname, int creation_flags, mode_t mode, AsyncOpenHandler &handler);

    /**
     * @brief The completion handler invoked by `Executor` once there is data to
     * read. This will call `read`, overwriting the specified buffer, and invoke
     * handler.
     * 
     * @param buffer The buffer to store the read data. Must be at least `size`
     * bytes long.
     * @param size The number of bytes to read.
     * @param handler The callback function to be invoked once the read 
     * operation has completed.
     */
    void read_completion_handler(Buffer &buffer, size_t size, AsyncIOHandler &handler);

    /**
     * @brief The asynchronous read operation. This will call `poll`. Once `poll`
     * `poll` returns, it will post the completion handler 
     * `read_completion_handler` to the `Executor`.
     * 
     * @param buffer The buffer to store the read data. Must be at least `size`
     * bytes long.
     * @param size The number of bytes to read.
     * @param handler The callback function to be invoked once the read 
     * operation has completed.
     */
    void read_async_operation(Buffer &buffer, size_t size, AsyncIOHandler &handler);

    /**
     * @brief The completion handler invoked by `Executor` once writing is 
     * possible. This will call `write`, writing the data in the specified
     * buffer to the file, and call handler.
     * 
     * @param buffer The buffer to write to the file. Must be at least `size`
     * bytes long.
     * @param size The number of bytes to write.
     * @param handler The callback function to be invoked once the write 
     * operation has completed.
     */
    void write_completion_handler(const Buffer &buffer, size_t size, AsyncIOHandler &handler);

    /**
     * @brief The asynchronous write operation. This will call `poll`. Once 
     * `poll` returns, it will post the completion handler 
     * `write_completion_handler` to the `Executor`.
     * 
     * @param buffer The buffer to write to the file. Must be at least `size`
     * bytes long.
     * @param size The number of bytes to write.
     * @param handler The callback function to be invoked once the write 
     * operation has completed.
     */
    void write_async_operation(const Buffer &buffer, size_t size, AsyncIOHandler &handler);
};

}  // namespace ipc
}  // namespace rix