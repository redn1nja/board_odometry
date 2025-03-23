#pragma once

#include <vector>

#include "ret_codes.h"
#include "Logger.h"

namespace vision {


typedef struct Attitude {
    float roll;
    float pitch;
    int yaw;
    bool operator==(const struct Attitude& other) const {
        return (std::abs(roll - other.roll) < 0.1f) && 
               (std::abs(pitch - other.pitch) < 0.1f) && 
               (yaw == other.yaw);
    }
    bool operator!=(const Attitude& other) const {
        return !(*this == other);
    }
} Attitude;


} // namespace vision