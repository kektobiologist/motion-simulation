#ifndef TRAJECTORYGENERATORS_HPP
#define TRAJECTORYGENERATORS_HPP
#include <tracking.hpp>
#include <functional>
#include "geometry.h"
using namespace std;
namespace TrajectoryGenerators {
Trajectory circleGenerator(double x, double y, double r, double startTheta, double f) {
    function<double(double)> xfunc = [=](double t)->double {
        return r*sin(2*PI*f*t + startTheta)+x;
    };
    function<double(double)> yfunc = [=](double t)->double {
        return r*cos(2*PI*f*t + startTheta)+y;
    };
    return Trajectory(xfunc, yfunc);
}
}
#endif // TRAJECTORYGENERATORS_HPP
