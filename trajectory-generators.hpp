#ifndef TRAJECTORYGENERATORS_HPP
#define TRAJECTORYGENERATORS_HPP
#include <tracking.hpp>
#include <functional>
#include "geometry.h"
#include "pose.h"
#include "velocity-profile.hpp"
#include "trajectory.hpp"
#include "splines.hpp"
#include "controlpoint-optimization.hpp"
#include "drawable.h"
using namespace std;
extern RenderArea *gRenderArea;
namespace TrajectoryGenerators {
Trajectory* circleGenerator(double x, double y, double r, double startTheta, double f) {
    function<double(double)> xfunc = [=](double t)->double {
        return (r*sin(2*PI*f*t + startTheta)+x)/fieldXConvert;
    };
    function<double(double)> yfunc = [=](double t)->double {
        return (r*cos(2*PI*f*t + startTheta)+y)/fieldXConvert;
    };
    return new Trajectory(xfunc, yfunc);
}

Trajectory *quinticBezierSplineGenerator(Pose start, Pose end, double vls, double vrs, double vle, double vre) {

    QuinticBezierSpline *p = new QuinticBezierSpline(start, end, vls, vrs, vle, vre);
    SplineTrajectory *st = new SplineTrajectory(p, vls, vrs, vle, vre);
    return st;
////    double k = 1/3.;
//    function<double(double)> xfunc = [=](double t)->double {
//        return st->x(t);
//    };
//    function<double(double)> yfunc = [=](double t)->double {
//        return st->y(t);
//    };
//    return new Trajectory(xfunc, yfunc);

}

Trajectory *ellipseGen(double x, double y, double a, double b, double startTheta, double f) {
    function<double(double)> xfunc = [=](double t)->double {
        return a*sin(2*PI*f*t + startTheta)+x;
    };
    function<double(double)> yfunc = [=](double t)->double {
        return b*cos(2*PI*f*t + startTheta)+y;
    };
    return new Trajectory(xfunc, yfunc);
}

Trajectory *cubic(Pose start, Pose end, double vls, double vrs, double vle, double vre) {
    CubicSpline *p = new CubicSpline(start, end);
    SplineTrajectory *st = new SplineTrajectory(p, vls, vrs, vle, vre);
    return st;
}

Trajectory *cubic2CP(Pose start, Pose end, double vls, double vrs, double vle, double vre) {

//    Pose cp1((start.x()*2+end.x())*1/3., (start.y()*2+end.y())*1/3., 0);
//    Pose cp2((start.x()+2*end.x())*1/3., (start.y()+2*end.y())*1/3., 0);
//    vector<Pose> midPoints;
//    midPoints.push_back(cp1);
//    midPoints.push_back(cp2);
//    CubicSpline *p = new CubicSpline(start, end, midPoints);
//    SplineTrajectory *st = new SplineTrajectory(p, vls, vrs, vle, vre);
    Trajectory *st = Optimization::cubicSpline2CPOptimization(start, end, vls, vrs, vle, vre);
    return st;
}
}
#endif // TRAJECTORYGENERATORS_HPP
