#pragma once

#ifndef RIX_UTIL_LOG_LEVEL
#define RIX_UTIL_LOG_LEVEL 1
#endif

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <mutex>

#include "rix/util/timing.hpp"

namespace rix {
namespace util {

/**
 * @brief A class for logging messages with different log levels.
 */
class Log {
   private:
    /**
     * @brief Log level enum. These values are used as template parameters to
     * the LogStream class. They will be checked against the value of
     * RIX_UTIL_LOG_LEVEL at compile time to determine if data should be sent
     * to stdout/stderr.
     *
     */
    enum Level { DEBUG, INFO, WARN, ERROR, FATAL };

    /**
     * @brief NullBuffer struct. This is used as a fake stream that writes
     * no data to stdout/stderr without accumulating data in a stream.
     *
     */
    struct NullBuffer : public std::streambuf {
        int overflow(int c) override { return c; }
    };

    /**
     * @brief static declaration of NullBuffer used by LogStream objects to
     * "write" to when the RIX_UTIL_LOG_LEVEL < level.
     *
     */
    inline static NullBuffer null_buffer{};

    inline static std::mutex mutex{};

    /**
     * @brief LogStream class. This class has a << operator that will append
     * header information to the data that is input to the stream. The level
     * template parameter is used to determine if data should be logged at
     * compile time. This minimizes runtime overhead when data should not be
     * logged.
     *
     * @tparam level
     */
    template <Level level>
    class LogStream {
       public:
        template <typename T>
        std::ostream &operator<<(const T &val) {
            if (level < RIX_UTIL_LOG_LEVEL) {
                return null_stream;
            }

            std::lock_guard<std::mutex> guard(mutex);
            std::string header = create_header();
            if (level > INFO) {
                return std::cerr << header << val;
            }
            return std::cout << header << val;
        }

       private:
        inline static std::ostream null_stream{&null_buffer};
        inline static std::mutex &mutex{Log::mutex};

        inline static std::string create_header() {
            return "[" + rix::util::now().to_string() + "] [" + bold + get_color_code(level) + get_level_str(level) +
                   reset_color + unbold + "] [" + bold + name + unbold + "] ";
        }
    };

   public:
    inline static void init(const std::string &name);

    /**
     * The public LogStream objects. These are used to log inforamtion at the
     * corresponding level. If RIX_UTIL_LOG_LEVEL is greater than the template
     * level parameter, then no data will be logged when used.
     *
     */
    inline static LogStream<Level::DEBUG> debug;
    inline static LogStream<Level::INFO> info;
    inline static LogStream<Level::WARN> warn;
    inline static LogStream<Level::ERROR> error;
    inline static LogStream<Level::FATAL> fatal;

   private:
    inline static const std::string reset_color = "\033[0m";
    inline static const std::string bold = "\x1b[1m";
    inline static const std::string unbold = "\x1b[22m";
    inline static std::string name;
    inline static bool is_init{false};

    inline static std::string get_color_code(Level level);
    inline static std::string get_level_str(Level level);
};

void Log::init(const std::string &name) {
    if (is_init) {
        return;
    }
    is_init = true;
    Log::name = name;
}

std::string Log::get_color_code(Level level) {
    switch (level) {
        case Level::DEBUG:
            return "\033[36m";  // Cyan
        case Level::INFO:
            return "\033[32m";  // Green
        case Level::WARN:
            return "\033[33m";  // Yellow
        case Level::ERROR:
            return "\033[31m";  // Red
        case Level::FATAL:
            return "\033[35m";  // Magenta
        default:
            return "\033[0m";  // Reset
    }
}

std::string Log::get_level_str(Level level) {
    switch (level) {
        case Level::DEBUG:
            return "DEBUG";  // Cyan
        case Level::INFO:
            return "INFO";  // Green
        case Level::WARN:
            return "WARN";  // Yellow
        case Level::ERROR:
            return "ERROR";  // Red
        case Level::FATAL:
            return "FATAL";  // Magenta
        default:
            return "";  // Reset
    }
}

}  // namespace util
}  // namespace rix