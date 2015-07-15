#ifndef COLLISIONCHECKING_H
#define COLLISIONCHECKING_H
// code for checking collision with arena walls
#include "splines.hpp"
#include "pose.h"
#include <vector>

namespace CollisionChecking {
struct LineSegment {
    double x1, y1, x2, y2;
    LineSegment(double x1, double y1, double x2, double y2): x1(x1), y1(y1), x2(x2), y2(y2) {}
    LineSegment(): x1(0), y1(0), x2(0), y2(0) {}
};

// returns array of u [atmost 3] of the intersection points of the cubic spline component
// given by coeffX and coeffY with the line segment ls, within u values of [u_low, u_high]
std::vector<double> cubic_LineSegmentIntersection(const double coeffX[4], const double coeffY[4], double u_low,
    double u_high, const LineSegment &ls);
std::vector<Pose> cubicSpline_LineSegmentIntersection(const CubicSpline &s, const LineSegment &ls);
}
#endif // COLLISIONCHECKING_H
