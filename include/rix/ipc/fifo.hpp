#pragma once

#include "rix/ipc/file.hpp"

namespace rix {
namespace ipc {

class Fifo : public File {
   public:
    /**
     * @brief Deletes a name from the filesystem. If that name was the
     * last link to a file and no processes have the file open, the file
     * is deleted and the space it was using is made available for reuse.
     *
     * @param name The path to the file
     * @return -1 on failure, 0 on success
     */
    static int remove(const std::string &name);

    /**
     * @brief Construct a new Fifo object. This will call `File::open`. If
     * successful, `is_open` will return true.
     * 
     * @param name The path to the FIFO special file.
     * @param creation_flags Bitmask of file creation flags.
     * @param mode The mode to open the file in.
     */
    Fifo(const std::string &name, int creation_flags, mode_t mode);

    /**
     * @brief Construct a new Fifo object.
     * 
     * @param name 
     * @param creation_flags 
     * @param mode 
     * @param handler 
     */
    Fifo(const std::string &name, int creation_flags, mode_t mode, const AsyncHandler &handler);

    /**
     * @brief Construct a new Fifo object. Sets fd to -1 by default.
     *
     */
    Fifo();

    Fifo(const Fifo &other);
    Fifo &operator=(const Fifo &other);
    ~Fifo();

    /**
     * @brief Remover class to ensure that file is unlinked before opening/creating
     * and unlinked again once Remover is destroyed. 
     * 
     */
    struct Remover {
        Remover(const std::string &name) : name(name) { Fifo::remove(name); }
        ~Remover() { Fifo::remove(name); }

       private:
        const std::string name;
    };

  private:
    void ctor_completion_handler(int status, int fd, const AsyncHandler &handler);

    using File::async_open;
    using File::open;
};

}  // namespace ipc
}  // namespace rix