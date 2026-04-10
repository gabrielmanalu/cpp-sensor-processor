#pragma once
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include "../core/PointCloud.hpp"

namespace sensor::io {

// Reads Semantic3D .txt files: space-delimited, no header.
// Column layout: x y z intensity r g b [semantic_label]
// intensity and semantic_label are ignored; r g b are loaded for PointXYZRGB.
template <typename T>
class Semantic3DReader {
public:
    explicit Semantic3DReader(std::string_view file_path)
        : stream_(std::make_unique<std::ifstream>(std::string(file_path),
                                                  std::ios::in | std::ios::binary)) {
        if (!stream_->is_open())
            throw std::runtime_error(std::string("Cannot open: ") + std::string(file_path));
    }

    // Returns nullptr when the file is exhausted.
    std::unique_ptr<core::PointCloud<T>> readChunk(std::size_t chunk_size) {
        auto cloud = std::make_unique<core::PointCloud<T>>(chunk_size);
        std::string line;
        std::size_t count = 0;

        while (count < chunk_size && std::getline(*stream_, line)) {
            if (line.empty()) continue;

            std::istringstream ss(line);
            T p{};
            float intensity = 0.0f;

            // x y z intensity
            if (!(ss >> p.x >> p.y >> p.z >> intensity))
                continue;   // skip malformed lines

            if constexpr (core::has_rgb_v<T>) {
                int r = 0, g = 0, b = 0;
                if (!(ss >> r >> g >> b)) continue;
                p.r = static_cast<uint8_t>(r);
                p.g = static_cast<uint8_t>(g);
                p.b = static_cast<uint8_t>(b);
            }

            cloud->addPoint(std::move(p));
            ++count;
        }

        return cloud->size() > 0 ? std::move(cloud) : nullptr;
    }

private:
    std::unique_ptr<std::ifstream> stream_;
};

} // namespace sensor::io
