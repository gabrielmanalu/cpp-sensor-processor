#pragma once
#include <iostream>
#include <mutex>
#include <chrono>
#include <ctime>
#include <iomanip>

namespace sensor::utils {

class Logger {
public:
    template <typename... Args>
    static void info(Args&&... args) { log("INFO ", std::forward<Args>(args)...); }

    template <typename... Args>
    static void warn(Args&&... args) { log("WARN ", std::forward<Args>(args)...); }

    template <typename... Args>
    static void error(Args&&... args) { log("ERROR", std::forward<Args>(args)...); }

private:
    template <typename... Args>
    static void log(const char* level, Args&&... args) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::system_clock::now();
        auto t   = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
#ifdef _WIN32
        localtime_s(&tm_buf, &t);
#else
        localtime_r(&t, &tm_buf);
#endif
        std::cout << "[" << std::put_time(&tm_buf, "%H:%M:%S") << "][" << level << "] ";
        (std::cout << ... << std::forward<Args>(args));
        std::cout << "\n";
    }

    static inline std::mutex mutex_;
};

} // namespace sensor::utils
