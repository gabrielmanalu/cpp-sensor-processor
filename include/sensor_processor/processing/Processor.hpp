#pragma once
#include <cstddef>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include "../core/PointCloud.hpp"
#include "ThreadPool.hpp"
#include "../utils/Logger.hpp"

namespace sensor::proc {

struct Stats {
    float min_x{0}, max_x{0}, mean_x{0};
    float min_y{0}, max_y{0}, mean_y{0};
    float min_z{0}, max_z{0}, mean_z{0};
    double avg_r{0}, avg_g{0}, avg_b{0};
    std::size_t count{0};
};

// Observer interface - implement to receive stats after each chunk.
class IStatsObserver {
public:
    virtual void onStats(const Stats& stats) = 0;
    virtual ~IStatsObserver()               = default;
};

template <typename T>
class Processor {
public:
    explicit Processor(std::size_t thread_count) : pool_(thread_count) {}

    void addObserver(std::shared_ptr<IStatsObserver> obs) {
        observers_.push_back(std::move(obs));
    }

    Stats computeParallelStats(const core::PointCloud<T>& cloud) {
        const auto& pts = cloud.points();
        const std::size_t n  = pts.size();
        if (n == 0) return {};

        const std::size_t num_threads = pool_.size();
        // Ceiling division so no points are skipped.
        const std::size_t chunk_size  = (n + num_threads - 1) / num_threads;

        std::vector<std::future<Stats>> futures;
        futures.reserve(num_threads);

        for (std::size_t i = 0; i < num_threads; ++i) {
            const std::size_t start = i * chunk_size;
            if (start >= n) break;
            const std::size_t end = std::min(start + chunk_size, n);

            futures.push_back(pool_.enqueue([&pts, start, end]() {
                Stats s;
                s.min_x = s.max_x = pts[start].x;
                s.min_y = s.max_y = pts[start].y;
                s.min_z = s.max_z = pts[start].z;

                for (std::size_t j = start; j < end; ++j) {
                    const auto& p = pts[j];

                    s.min_x   = std::min(s.min_x, p.x);
                    s.max_x   = std::max(s.max_x, p.x);
                    s.mean_x += p.x;

                    s.min_y   = std::min(s.min_y, p.y);
                    s.max_y   = std::max(s.max_y, p.y);
                    s.mean_y += p.y;

                    s.min_z   = std::min(s.min_z, p.z);
                    s.max_z   = std::max(s.max_z, p.z);
                    s.mean_z += p.z;

                    if constexpr (core::has_rgb_v<T>) {
                        s.avg_r += p.r;
                        s.avg_g += p.g;
                        s.avg_b += p.b;
                    }
                }
                s.count = end - start;
                return s;
            }));
        }

        // Merge per-thread results into a single global Stats.
        Stats global{};
        bool first = true;
        for (auto& f : futures) {
            const Stats s = f.get();
            if (s.count == 0) continue;
            if (first) {
                global = s;   // initialises min/max from the first chunk
                first  = false;
            } else {
                global.min_x = std::min(global.min_x, s.min_x);
                global.max_x = std::max(global.max_x, s.max_x);
                global.mean_x += s.mean_x;

                global.min_y = std::min(global.min_y, s.min_y);
                global.max_y = std::max(global.max_y, s.max_y);
                global.mean_y += s.mean_y;

                global.min_z = std::min(global.min_z, s.min_z);
                global.max_z = std::max(global.max_z, s.max_z);
                global.mean_z += s.mean_z;

                global.avg_r  += s.avg_r;
                global.avg_g  += s.avg_g;
                global.avg_b  += s.avg_b;
                global.count  += s.count;
            }
        }

        if (global.count > 0) {
            const auto inv = 1.0f / static_cast<float>(global.count);
            global.mean_x *= inv;
            global.mean_y *= inv;
            global.mean_z *= inv;

            if constexpr (core::has_rgb_v<T>) {
                const double dinv = 1.0 / static_cast<double>(global.count);
                global.avg_r *= dinv;
                global.avg_g *= dinv;
                global.avg_b *= dinv;
            }
        }

        for (auto& obs : observers_) obs->onStats(global);
        return global;
    }

private:
    ThreadPool pool_;
    std::vector<std::shared_ptr<IStatsObserver>> observers_;
};

} // namespace sensor::proc
