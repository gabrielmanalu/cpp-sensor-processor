#pragma once
#include <vector>
#include <memory>
#include "PointTypes.hpp"

namespace sensor::core {

template <typename T>
class PointCloud {
public:
    using PointType = T;
    
    PointCloud() = default;
    explicit PointCloud(size_t reserve_size) { points_.reserve(reserve_size); }

    void addPoint(const T& point) { points_.push_back(point); }
    void addPoint(T&& point) { points_.push_back(std::move(point)); }

    const std::vector<T>& points() const noexcept { return points_; }
    std::vector<T>& points() noexcept { return points_; }
    
    size_t size() const noexcept { return points_.size(); }
    bool empty() const noexcept { return points_.empty(); }
    void clear() { points_.clear(); }

private:
    std::vector<T> points_;
};

} // namespace sensor::core