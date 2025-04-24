#ifndef ANGULAR_H
#define ANGULAR_H

#include <math.h>

inline double rotate(double ang) {
    return std::fmod(ang + 2 * M_PI, 2 * M_PI) - M_PI;
}


inline double wrap_PI(double angle) {
    while (angle > 2 * M_PI)
        angle -= 2 * M_PI;

    while (angle < 0)
        angle += 2 * M_PI;
    return angle;
}

#endif //ANGULAR_H
