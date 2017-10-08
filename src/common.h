#ifndef COMMON_H_
#define COMMON_H_

#include <math.h>
#include <iostream>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
static inline double deg2rad(double x) { return x * pi() / 180; }
static inline double rad2deg(double x) { return x * 180 / pi(); }
static inline double mph2mps(double x) { return x * 1609.34 / 60 / 60; }

static inline double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

#endif  // COMMON_H_
