#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <optional>
#include "../core/PointCloud.hpp"
#include "../utils/Logger.hpp"

namespace sensor::io {

template <typename T>
class CsvReader {
public:
    explicit CsvReader(std::string_view file_path) 
        : stream_(std::make_unique<std::ifstream>(file_path.data())) {
        if (!stream_->is_open()) {
            throw std::runtime_error("Could not open file");
        }
        // Skip header
        std::string dummy;
        std::getline(*stream_, dummy);
    }

    // Reads in chunks to avoid massive memory spikes
    std::unique_ptr<core::PointCloud<T>> readChunk(size_t chunk_size) {
        auto cloud = std::make_unique<core::PointCloud<T>>();
        std::string line;
        size_t count = 0;

        while (count < chunk_size && std::getline(*stream_, line)) {
            std::stringstream ss(line);
            std::string val;
            T p;
            
            auto next_val = [&]() { std::getline(ss, val, ','); return std::stof(val); };

            p.x = next_val();
            p.y = next_val();
            p.z = next_val();

            if constexpr (core::has_rgb_v<T>) {
                p.r = static_cast<uint8_t>(next_val());
                p.g = static_cast<uint8_t>(next_val());
                p.b = static_cast<uint8_t>(next_val());
            }

            cloud->addPoint(std::move(p));
            count++;
        }
        return cloud->size() > 0 ? std::move(cloud) : nullptr;
    }

private:
    std::unique_ptr<std::ifstream> stream_;
};

} // namespace sensor::io