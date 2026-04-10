#pragma once
#include <chrono>

namespace sensor::utils {

class Timer {
public:
    Timer() : start_(clock::now()) {}

    // Returns elapsed seconds since construction or last reset().
    double elapsed() const {
        return std::chrono::duration<double>(clock::now() - start_).count();
    }

    void reset() { start_ = clock::now(); }

private:
    using clock = std::chrono::high_resolution_clock;
    clock::time_point start_;
};

} // namespace sensor::utils
