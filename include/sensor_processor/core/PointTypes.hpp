#pragma once
#include <cstdint>
#include <type_traits>

namespace sensor::core {

struct PointXYZ {
    float x{0.0f}, y{0.0f}, z{0.0f};
};

struct PointXYZRGB : public PointXYZ {
    uint8_t r{0}, g{0}, b{0};
};

// Traits for compile-time introspection
template <typename T>
struct has_rgb : std::false_type {};

template <>
struct has_rgb<PointXYZRGB> : std::true_type {};

template <typename T>
inline constexpr bool has_rgb_v = has_rgb<T>::value;

} // namespace sensor::core