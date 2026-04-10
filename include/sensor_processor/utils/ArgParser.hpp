#pragma once
#include <string>
#include <string_view>
#include <vector>
#include <algorithm>

namespace sensor::utils {

class ArgParser {
public:
    ArgParser(int argc, char** argv) : args_(argv + 1, argv + argc) {}

    // Returns the value following `flag`, or `default_val` if not present.
    std::string getOption(std::string_view flag, std::string_view default_val = "") const {
        for (auto it = args_.cbegin(); it != args_.cend(); ++it) {
            if (*it == flag) {
                if (auto next = std::next(it); next != args_.cend())
                    return *next;
            }
        }
        return std::string(default_val);
    }

    // Returns true if `flag` is present anywhere in the argument list.
    bool hasFlag(std::string_view flag) const {
        return std::any_of(args_.cbegin(), args_.cend(),
                           [&](const std::string& s) { return s == flag; });
    }

private:
    std::vector<std::string> args_;
};

} // namespace sensor::utils
