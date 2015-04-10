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
using namespace std;

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
    SplineTrajectory *st = new SplineTrajectory(*p, vls, vrs, vle, vre);
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
    botPosEnd.x_ = predictedBallPos.x;
    botPosEnd.y_ = predictedBallPos.y;
    double theta = Vector2D<int>::angle(goalCentre, predictedBallPos);
    botPosEnd.theta_ = theta;
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


Trajectory *cubic(Pose start, Pose end) {
    double d = sqrt((start.x() - end.x())*(start.x() - end.x()) + (start.y() - end.y())*(start.y() - end.y()));
    double k = 1300. / d;
    double x1 = start.x();
    double x2 = end.x();
    double y1 = start.y();
    double y2 = end.y();
    double th1 = start.theta();
    double th2 = end.theta();
    function<double(double)> xfunc = [=](double t)->double {
        double u = k*t;
        double a1 = d * cos(th2) + d * cos(th1) - 2 * (x2 - x1);
        double a2 = 3 * (x2 - x1) - d * cos(th2) - 2 * d * cos(th1);
        double a3 = d * cos(th1);
        double a4 = x1;
        return a1 * u * u * u + a2 * u * u + a3 * u + a4;
    };
    function<double(double)> yfunc = [=](double t)->double {
        double u = k*t;
        double b1 = d * sin(th2) + d * sin(th1) - 2 * (y2 - y1);
        double b2 = 3 * (y2 - y1) - d * sin(th2) - 2 * d * sin(th1);
        double b3 = d * sin(th1);
        double b4 = y1;
        return b1 * u * u * u + b2 * u * u + b3 * u + b4;
    };
    Trajectory traj(xfunc, yfunc);
    double maxvw=0, maxvl=0, maxvr =0;
    double i=0.0;
    while( i < 1/k){
    double v = traj.v(i);
    double w = traj.thetad(i);
    v /=Constants::fieldXConvert;
    double vl = v - Constants::d*w/2;
    double vr = v + Constants::d*w/2;
    if( vl > maxvl)
           maxvl = vl;
    if(maxvr < vr)
        maxvr = vr;
    if(maxvw < fabs(v*w))
        maxvw=fabs(v*w);
    i+=0.2;
    }
    qDebug() << maxvw << " " << maxvl << " " << maxvr << endl;
    return new Trajectory(traj);
}
}
#endif // TRAJECTORYGENERATORS_HPP
