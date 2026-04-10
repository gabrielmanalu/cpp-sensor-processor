#include <iostream>
#include "sensor_processor/core/PointCloud.hpp"
#include "sensor_processor/io/CsvReader.hpp"
#include "sensor_processor/processing/Processor.hpp"
#include "sensor_processor/utils/Timer.hpp"
#include "sensor_processor/utils/ArgParser.hpp"

using namespace sensor;

template <typename T>
void run_pipeline(std::string_view path, size_t threads) {
    utils::Timer timer;
    io::CsvReader<T> reader(path);
    proc::Processor<T> processor(threads);
    
    size_t total_points = 0;
    
    while (auto cloud = reader.readChunk(100000)) {
        total_points += cloud->size();
        auto stats = processor.computeParallelStats(*cloud);
        
        utils::Logger::info("Processed chunk. Local Mean X: ", stats.mean_x);
    }
    
    double elapsed = timer.elapsed();
    utils::Logger::info("Total Points: ", total_points);
    utils::Logger::info("Time: ", elapsed, "s");
    utils::Logger::info("Throughput: ", static_cast<double>(total_points) / elapsed, " pts/sec");
}

int main(int argc, char** argv) {
    utils::ArgParser args(argc, argv);
    
    std::string file = args.getOption("--file", "data/mock_sensor_xyz.csv");
    size_t threads = std::stoul(args.getOption("--threads", "4"));
    bool is_rgb = args.hasFlag("--rgb");

    try {
        if (is_rgb) {
            run_pipeline<core::PointXYZRGB>(file, threads);
        } else {
            run_pipeline<core::PointXYZ>(file, threads);
        }
    } catch (const std::exception& e) {
        utils::Logger::error(e.what());
        return 1;
    }

    return 0;
}