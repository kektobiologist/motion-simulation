#ifndef TESTS_HPP
#define TESTS_HPP
#include "pose.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <vector>
#include "splines.hpp"
#include "arclength-param.hpp"
using namespace std;
// ONLY include in dialog.cpp
namespace Tests {
Pose randomPose() {
    double x = (rand()/(double)RAND_MAX)*HALF_FIELD_MAXX;
    x *= 2*(rand()%2)-1;
    double y = (rand()/(double)RAND_MAX)*HALF_FIELD_MAXY;
    y *= 2*(rand()%2)-1;
    double theta = normalizeAngle((rand()/(double)RAND_MAX)*M_PI*2.);
    return Pose(x, y, theta);
}

Spline randomCubicSpline(int nCP) {
    // create random start/ end pose
    Pose start = randomPose(), end = randomPose();
    // add nCP random midPoints;
    vector<Pose> midPoints;
    for (int i = 0; i < nCP; i++) {
        midPoints.push_back(randomPose());
    }
    return CubicSpline(start, end, midPoints);
}

void arclengthParam_test() {
    // create a random spline
    Spline p = randomCubicSpline(4);
    Integration::computeInverseBezierMatrices(p);
}
}
#endif // TESTS_HPP
