#pragma once
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string_view>
#include "../core/PointCloud.hpp"

namespace sensor::io {

template <typename T>
class JsonExporter {
public:
    static void save(const core::PointCloud<T>& cloud, std::string_view filename,
                     std::size_t max_points = 50000) {
        std::ofstream file(filename.data());
        if (!file.is_open())
            throw std::runtime_error(std::string("Cannot write: ") + std::string(filename));
        file << "[\n";

        const auto& pts = cloud.points();
        // Stride-based downsampling so the JSON stays renderable in-browser.
        const std::size_t stride = std::max<std::size_t>(1, pts.size() / max_points);

        for (std::size_t i = 0; i < pts.size(); i += stride) {
            const auto& p = pts[i];
            file << "  {\"x\":" << p.x << ",\"y\":" << p.y << ",\"z\":" << p.z;
            
            if constexpr (core::has_rgb_v<T>) {
                file << ",\"r\":" << (int)p.r << ",\"g\":" << (int)p.g << ",\"b\":" << (int)p.b;
            }
            
            file << "}" << (i + stride < pts.size() ? "," : "") << "\n";
        }
        file << "]";
    }
};

} // namespace sensor::io