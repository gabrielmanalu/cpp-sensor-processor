#include <cstddef>
#include <iostream>
#include <string>
#include <string_view>
#include "sensor_processor/core/PointCloud.hpp"
#include "sensor_processor/io/CsvReader.hpp"
#include "sensor_processor/io/Semantic3DReader.hpp"
#include "sensor_processor/io/JsonExporter.hpp"
#include "sensor_processor/processing/Processor.hpp"
#include "sensor_processor/utils/ArgParser.hpp"
#include "sensor_processor/utils/Logger.hpp"
#include "sensor_processor/utils/Timer.hpp"

using namespace sensor;

// Returns the lowercased file extension including the dot, e.g. ".txt".
static std::string file_ext(std::string_view path) {
    auto pos = path.rfind('.');
    if (pos == std::string_view::npos) return "";
    std::string ext(path.substr(pos));
    for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext;
}

// ---- pipeline templated on both point type T and reader type Reader ----------
template <typename T, typename Reader>
void run_pipeline(std::string_view path, std::size_t threads, std::string_view output,
                  std::size_t chunk_size) {
    utils::Timer timer;
    Reader reader(path);
    proc::Processor<T> processor(threads);
    core::PointCloud<T> merged;

    std::size_t total_points  = 0;
    std::size_t chunk_index   = 0;

    while (auto cloud = reader.readChunk(chunk_size)) {
        total_points += cloud->size();
        ++chunk_index;

        auto stats = processor.computeParallelStats(*cloud);

        // Progress line every chunk so the user knows it's alive on large files.
        utils::Logger::info(
            "Chunk ", chunk_index,
            " | pts=", cloud->size(),
            " | total=", total_points,
            " | mean_x=", stats.mean_x,
            " | elapsed=", timer.elapsed(), "s");

        for (const auto& p : cloud->points())
            merged.addPoint(p);
    }

    const double elapsed = timer.elapsed();

    utils::Logger::info("--- Done ---");
    utils::Logger::info("Total points : ", total_points);
    utils::Logger::info("Time         : ", elapsed, " s");
    utils::Logger::info("Throughput   : ",
                        static_cast<double>(total_points) / elapsed, " pts/s");

    utils::Logger::info("Writing visualizer data to: ", output);
    sensor::io::JsonExporter<T>::save(merged, output);
    utils::Logger::info("Saved.");
}

// ---- dispatch on format + point type ----------------------------------------
template <typename T>
void dispatch(std::string_view path, std::string_view ext,
              std::size_t threads, std::string_view output, std::size_t chunk_size) {
    if (ext == ".txt" || ext == ".pts" || ext == ".xyz") {
        utils::Logger::info("Format: Semantic3D (space-delimited, no header)");
        run_pipeline<T, io::Semantic3DReader<T>>(path, threads, output, chunk_size);
    } else {
        utils::Logger::info("Format: CSV (comma-delimited, with header)");
        run_pipeline<T, io::CsvReader<T>>(path, threads, output, chunk_size);
    }
}

// -----------------------------------------------------------------------------
int main(int argc, char** argv) {
    utils::ArgParser args(argc, argv);

    const std::string  file       = args.getOption("--file",    "data/mock_sensor_xyz.csv");
    const std::string  output     = args.getOption("--output",  "vis/viz_data.json");
    const std::size_t  threads    = std::stoul(args.getOption("--threads", "4"));
    const std::size_t  chunk_size = std::stoul(args.getOption("--chunk",   "200000"));
    const bool         is_rgb     = args.hasFlag("--rgb");
    const std::string  ext        = file_ext(file);

    utils::Logger::info("File     : ", file);
    utils::Logger::info("Threads  : ", threads);
    utils::Logger::info("Chunk sz : ", chunk_size);
    utils::Logger::info("RGB mode : ", is_rgb ? "yes" : "no");

    try {
        if (is_rgb)
            dispatch<core::PointXYZRGB>(file, ext, threads, output, chunk_size);
        else
            dispatch<core::PointXYZ>(file, ext, threads, output, chunk_size);
    } catch (const std::exception& e) {
        utils::Logger::error(e.what());
        return 1;
    }

    return 0;
}
