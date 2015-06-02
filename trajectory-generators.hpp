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
// doesn't care about units, returns the answer in the same units of the input.
/*
Vector2D<double> predictBallPose(Vector2D<double> ballPos, Vector2D<double> ballVel, double timeOfPrediction){
    Vector2D<double> finalBallPos;
    finalBallPos.x = ballPos.x + timeOfPrediction*ballVel.x;
    finalBallPos.y = ballPos.y + timeOfPrediction*ballVel.y;
    return finalBallPos;
}
*/

Trajectory *ellipseGen(double x, double y, double a, double b, double startTheta, double f) {

    function<double(double)> xfunc = [=](double t)->double {
        return (a*sin(2*PI*f*t + startTheta)+x)/fieldXConvert;
    };
    function<double(double)> yfunc = [=](double t)->double {
        return (b*cos(2*PI*f*t + startTheta)+y)/fieldXConvert;
    };
    return new Trajectory(xfunc, yfunc);
}

SplineTrajectory *cubic(Pose start, Pose end, double vls, double vrs, double vle, double vre, vector<Pose> midPoints = vector<Pose>()) {
    CubicSpline *p = new CubicSpline(start, end, midPoints);

//    p->maxk();
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
// ballPos: strategy coordinates
// ballVel: strategy coordinates per second
/*
SplineTrajectory* ballInterception(Pose botPosStart, Vector2D<double> ballPos, Vector2D<double> ballVel){
    Vector2D<double> predictedBallPos;
    double error = 0.1;
    double T2 = 6.0;
    double T1 = 0.0;
    double d = 0.0;
    Vector2D<double> goalCentre(-HALF_FIELD_MAXX, 0);
    SplineTrajectory *st = NULL;
    while (1) {
        // predictedBallPos: strategy coordinates
        double mid = (T1+T2)/2;
        predictedBallPos = predictBallPose(ballPos, ballVel, mid);
        double endTheta = atan2(goalCentre.y - predictedBallPos.y, goalCentre.x - predictedBallPos.x);
        Pose endPose(predictedBallPos.x, predictedBallPos.y, endTheta);
        if (st)
            delete st;
        // add a cp behind the ball pos, distance of 500
        Pose cp1(predictedBallPos.x+500*cos(endTheta+M_PI), predictedBallPos.y+500*sin(endTheta+M_PI), 0);
        vector<Pose> midPoints;
        midPoints.push_back(cp1);
        st = cubic(botPosStart, endPose, 0, 0, 50, 50, midPoints);
        double t = st->totalTime();
        qDebug() << "mid = " << mid << ", t = " << t;
        if (fabs(t-mid) < error)
            break;
        if (t > mid) {
            T1 = mid;
        } else if (t < mid) {
            T2 = mid;
        }
        if (fabs(T2-T1) < error) {
            qDebug() << "T2, T1 almost same = " << T1 <<", t = " << t;
            break;
        }
    }
    return st;
}
*/
}

#endif // TRAJECTORYGENERATORS_HPP
