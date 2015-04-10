#ifndef TRAJECTORYGENERATORS_HPP
#define TRAJECTORYGENERATORS_HPP
#include <tracking.hpp>
#include <functional>
#include "geometry.h"
#include <QDebug>
#include <math.h>
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

Vector2D<int> predictBallPose(Vector2D<int> ballPos, Vector2D<int> ballVel, double timeOfPrediction){
    Vector2D<int> finalBallPos;
    finalBallPos.x = ballPos.x + timeOfPrediction*ballVel.x;
    finalBallPos.y = ballPos.y + timeOfPrediction*ballVel.y;
    return finalBallPos;
}

Trajectory ballInterception(Pose botPosStart, Vector2D<int> ballPos, Vector2D<int> ballVel){
    Pose botPosEnd;
    Vector2D<int> predictedBallPos;
    double error = 0.1;
    double T2 = 6.0;
    double T1 = 0.0;
    double d = 0.0;
    Vector2D<int> goalCentre(HALF_FIELD_MAXX, 0);
    while (1) {
        predictedBallPos = predictBallPose(ballPos, ballVel, (T1 + T2) / 2);
        d = sqrt((botPosStart.x() - predictedBallPos.x)*(botPosStart.x() - predictedBallPos.x) + (botPosStart.y() - predictedBallPos.y)*(botPosStart.y() - predictedBallPos.y));
        if (d / 1300 > (T1 + T2) / 2) {
            T1 = (T1 + T2) / 2;
        } else if (d / 1300 < (T1 + T2) / 2) {
            T2 = (T1 + T2) / 2;
        } else if (abs(d / 1300 - (T1 + T2) / 2) < error) {
            break;
        }
    }
    botPosEnd.setX(predictedBallPos.x);
    botPosEnd.setY(predictedBallPos.y);
    double theta = Vector2D<int>::angle(goalCentre, predictedBallPos);
    botPosEnd.setTheta(theta);
}

Trajectory *ellipseGen(double x, double y, double a, double b, double startTheta, double f) {

    function<double(double)> xfunc = [=](double t)->double {
        return (a*sin(2*PI*f*t + startTheta)+x)/fieldXConvert;
    };
    function<double(double)> yfunc = [=](double t)->double {
        return (b*cos(2*PI*f*t + startTheta)+y)/fieldXConvert;
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
//    static PointDrawable *pt1 = NULL, *pt2 = NULL;
//    if (pt1)
//        delete pt1;
//    if (pt2)
//        delete pt2;
//    pt1 = new PointDrawable(QPointF(cp1.x(), cp1.y()), gRenderArea);
//    pt2 = new PointDrawable(QPointF(cp2.x(), cp2.y()), gRenderArea);
//    CubicSpline *p = new CubicSpline(start, end, midPoints);
//    SplineTrajectory *st = new SplineTrajectory(p, vls, vrs, vle, vre);
    Trajectory *st = Optimization::cubicSpline2CPOptimization(start, end, vls, vrs, vle, vre);
    return st;
}
}
#endif // TRAJECTORYGENERATORS_HPP
