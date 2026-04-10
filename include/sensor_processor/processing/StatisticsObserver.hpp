#pragma once
#include <fstream>
#include <string>
#include <iomanip>
#include "Processor.hpp"
#include "../utils/Logger.hpp"

namespace sensor::proc {

// Logs stats to console after each chunk via the Logger utility.
class ConsoleStatsObserver : public IStatsObserver {
public:
    void onStats(const Stats& s) override {
        utils::Logger::info(
            "  x[", s.min_x, ", ", s.max_x, "] mean=", s.mean_x,
            " | y[", s.min_y, ", ", s.max_y, "] mean=", s.mean_y,
            " | z[", s.min_z, ", ", s.max_z, "] mean=", s.mean_z,
            " | n=", s.count);
    }
};

// Appends a CSV row of stats for each chunk to a file for later analysis.
class CsvStatsObserver : public IStatsObserver {
public:
    explicit CsvStatsObserver(const std::string& path)
        : out_(path, std::ios::app) {
        if (out_.tellp() == 0)
            out_ << "count,min_x,max_x,mean_x,min_y,max_y,mean_y,min_z,max_z,mean_z\n";
    }

    void onStats(const Stats& s) override {
        out_ << s.count      << ','
             << s.min_x      << ',' << s.max_x  << ',' << s.mean_x << ','
             << s.min_y      << ',' << s.max_y  << ',' << s.mean_y << ','
             << s.min_z      << ',' << s.max_z  << ',' << s.mean_z << '\n';
    }

private:
    std::ofstream out_;
};

} // namespace sensor::proc
