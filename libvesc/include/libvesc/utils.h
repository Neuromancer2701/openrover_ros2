
#ifndef VESC_UTILS_H
#define VESC_UTILS_H

#include <vector>
#include <numeric>
#include <cstddef>

namespace {
    using std::vector;
    using std::iota;
}

using std::uint8_t;
using std::uint16_t;
using std::uint32_t;

namespace vesc {
/**
 * @brief Contains utility functions for the VESC library.
 * 
 * This namespace provides common helper functions, primarily for casting between types,
 * used throughout the VESC communication interface.
 */
namespace utils
{
    /**
     * @brief Casts a value to uint8_t.
     * @tparam T The type of the input value.
     * @param data The value to cast.
     * @return The value cast to uint8_t.
     */
    template<class T>
    auto castu8(T data){return static_cast<uint8_t>(data);}

    /**
     * @brief Casts a value to uint16_t.
     * @tparam T The type of the input value.
     * @param data The value to cast.
     * @return The value cast to uint16_t.
     */
    template<class T>
    auto castu16(T data){return static_cast<uint16_t>(data);}

    /**
     * @brief Casts a value to uint32_t.
     * @tparam T The type of the input value.
     * @param data The value to cast.
     * @return The value cast to uint32_t.
     */
    template<class T>
    auto castu32(T data){return static_cast<uint32_t>(data);}

    /**
     * @brief Casts a value to double.
     * @tparam T The type of the input value.
     * @param data The value to cast.
     * @return The value cast to double.
     */
    template<class T>
    auto castdouble(T data){return static_cast<double>(data);}

}; // namespace utils
} // namespace vesc

#endif //VESC_UTILS_H
