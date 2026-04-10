#include <cstddef>
#include <iostream>
#include "sensor_processor/core/PointCloud.hpp"
#include "sensor_processor/io/CsvReader.hpp"
#include "sensor_processor/processing/Processor.hpp"
#include "sensor_processor/utils/Timer.hpp"
#include "sensor_processor/utils/ArgParser.hpp"
#include "sensor_processor/io/JsonExporter.hpp"

using namespace sensor;

template <typename T>
void run_pipeline(std::string_view path, std::size_t threads, std::string_view output) {
    utils::Timer timer;
    io::CsvReader<T> reader(path);
    proc::Processor<T> processor(threads);
    sensor::core::PointCloud<T> merged;
    
    std::size_t total_points = 0;
    
    while (auto cloud = reader.readChunk(100000)) {
        total_points += cloud->size();
        auto stats = processor.computeParallelStats(*cloud);
        
        utils::Logger::info("Processed chunk. Local Mean X: ", stats.mean_x);
        for(const auto& p : cloud->points()){
            merged.addPoint(p);
        }
    }
    
    double elapsed = timer.elapsed();
    sensor::io::JsonExporter<T>::save(merged, output);
    utils::Logger::info("Saved visualization to: ", output);
    utils::Logger::info("Total Points: ", total_points);
    utils::Logger::info("Time: ", elapsed, "s");
    utils::Logger::info("Throughput: ", static_cast<double>(total_points) / elapsed, " pts/sec");
}

int main(int argc, char** argv) {
    utils::ArgParser args(argc, argv);
    
    std::string file   = args.getOption("--file",    "data/mock_sensor_xyz.csv");
    std::string output = args.getOption("--output",  "vis/viz_data.json");
    std::size_t threads = std::stoul(args.getOption("--threads", "4"));
    bool is_rgb = args.hasFlag("--rgb");

    try {
        if (is_rgb) {
            run_pipeline<core::PointXYZRGB>(file, threads, output);
        } else {
            run_pipeline<core::PointXYZ>(file, threads, output);
        }
    } catch (const std::exception& e) {
        utils::Logger::error(e.what());
        return 1;
    }

    return 0;
}