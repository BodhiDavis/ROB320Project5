#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace rix {
namespace util {

/**
 * @brief A struct for date and time.
 */
struct DateTime {
    int year;         ///< Year component of the date and time.
    int month;        ///< Month component of the date and time.
    int day;          ///< Day component of the date and time.
    int hour;         ///< Hour component of the date and time.
    int minute;       ///< Minute component of the date and time.
    int second;       ///< Second component of the date and time.
    int millisecond;  ///< Millisecond component of the date and time.

    /**
     * @brief Converts the DateTime to a string.
     * @return The string representation of the DateTime.
     */
    std::string to_string() const;

    /**
     * @brief Stream insertion operator for DateTime.
     * @param os The output stream.
     * @param dt The DateTime to insert into the stream.
     * @return The output stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const DateTime &dt) {
        os << dt.to_string();
        return os;
    }
};

/**
 * @brief Get the current date and time.
 * @return The current date and time as a DateTime struct.
 */
DateTime now();

/**
 * @brief Sleep for a given number of milliseconds.
 * @param ms The number of milliseconds to sleep.
 */
void sleep(int ms);

/**
 * @brief Get the current time in milliseconds.
 * @return The current time in milliseconds.
 */
int64_t millis();

/**
 * @brief Get the current time in microseconds.
 * @return The current time in microseconds.
 */
int64_t micros();

/**
 * @brief Get the current time in nanoseconds.
 * @return The current time in nanoseconds.
 */
int64_t nanos();

/**
 * @brief A class for measuring time.
 */
class Timer {
   public:
    /**
     * @brief Constructs a Timer object.
     */
    Timer();

    /**
     * @brief Starts the timer.
     */
    void start();

    /**
     * @brief Stops the timer.
     */
    void stop();

    /**
     * @brief Gets the elapsed time in milliseconds.
     * @return The elapsed time in milliseconds.
     */
    int64_t elapsed_ms() const;

    /**
     * @brief Gets the elapsed time in microseconds.
     * @return The elapsed time in microseconds.
     */
    int64_t elapsed_us() const;

    /**
     * @brief Gets the elapsed time in nanoseconds.
     * @return The elapsed time in nanoseconds.
     */
    int64_t elapsed_ns() const;

   private:
    using _ms = std::chrono::milliseconds;
    using _us = std::chrono::microseconds;
    using _ns = std::chrono::nanoseconds;

    std::chrono::time_point<std::chrono::high_resolution_clock> _start;  ///< Start time point.
    std::chrono::time_point<std::chrono::high_resolution_clock> _stop;   ///< Stop time point.
};

/**
 * @brief A class for setting the rate of a loop.
 * 
 * @example shared_mutex.cpp
 * @example tcp_client.cpp
 * @example tcp_client_noblock.cpp
 * @example udp_sender_multicast.cpp
 * @example udp_receiver.cpp
 * @example web_client.cpp
 */
class Rate {
   public:
    /**
     * @brief Constructs a Rate object.
     * @param hz The frequency in hertz.
     */
    Rate(double hz);

    /**
     * @brief Sleeps for the time required to maintain the rate.
     * @return True if the sleep duration was met, false otherwise.
     */
    bool sleep();

   private:
    int64_t _period_ns;  ///< The period in nanoseconds.
    std::chrono::time_point<std::chrono::high_resolution_clock> _start;  ///< Start time point.
};

}  // namespace util
}  // namespace rix