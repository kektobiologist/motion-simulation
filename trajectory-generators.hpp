#ifndef TRAJECTORYGENERATORS_HPP
#define TRAJECTORYGENERATORS_HPP
#include <tracking.hpp>
#include <functional>
#include "geometry.h"
#include "pose.h"
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

Trajectory quinticBezierSplineGenerator(Pose start, Pose end, double vls, double vrs, double vle, double vre) {
    double d = dist(start, end);
    double d_2 = d*10.;
//    d = 1;
    // ts = tangent at start = (dx/du(0), dy/du(0)) = (d*cos(starttheta), d*sin(starttheta))
    Point2D<double> ts;
    ts.x = d*cos(start.theta());
    ts.y = d*sin(start.theta());
//    ts = Point2D<double>(0,0);
    qDebug() << "ts = " << ts.x << ts.y;
    // te = tangent at end = (dx/du(1), dy/du(1)) = (d*cos(endtheta), d*sin(endtheta))
    Point2D<double> te;
    te.x = d*cos(end.theta());
    te.y = d*sin(end.theta());
//    te = Point2D<double>(0,0);
    qDebug() << "te = " << te.x << te.y;
    // as = second derivaives at start (d2x/du2(0), d2y/du2(0))
    // solved using:
    // ks = a*ydd - b*xdd   (a,b are functions of xd(0) = ts_x and yd(0) = ts_y)
    // d_2^2 = ydd^2 + xdd^2
    // relies on ts_x = d*cos(theta), ts_y = d*sin(theta)
    // ks is starting curvature, found as ks = omega/v
    // the solution is: xdd = cos(t)*d_2, ydd = sin(t)*d_2, t = asin(d*d*ks/d_2)+theta
    Point2D<double> as;
    {
        double ks;
        if (vrs+vls != 0)
            ks = (vrs-vls)/(double)(vrs+vls)*2./Constants::d;
        else
            ks = 0;
        // convert ks to strategy coordinates
        ks /= Constants::fieldXConvert;
        qDebug() << "d*ks = " << d*ks << ", radius = " << 1/ks;
        double t = asin(d*d*ks/d_2)+start.theta();
        as.x = cos(t)*d_2;
        as.y = sin(t)*d_2;
        // debugging,setting 0
//        as = Point2D<double>(0,0);
        qDebug() << "ks = " << ks << ", as = " << as.x << as.y;
    }
    // similarly find ae_x, ae_y
    Point2D<double> ae;
    {
        double ke;
        if (vre+vle!=0)
            ke = (vre-vle)/(double)(vre+vle)*2./Constants::d;
        else
            ke = 0;
        // convert ke to strategy coordinaetes
        ke /= Constants::fieldXConvert;
        qDebug() << "d*ke = " << d*ke << ", radius = " << 1/ke;
        double t = asin(d*d*ke/d_2)+end.theta();
        ae.x = d_2*cos(t);
        ae.y = d_2*sin(t);
        // debugging, setting 0
//        ae = Point2D<double>(0,0);
        qDebug() << "ke = " << ke << ", ae = " << ae.x << ae.y;
    }
    // P0 = start
    Point2D<double> P0(start.x(), start.y());
    // P1 = 1/5*ts + P0;
    Point2D<double> P1 = 0.2*ts+P0;
    // P2 = 1/20*as+2*P1-P0
    Point2D<double> P2 = 1/20.*as+2*P1-P0;
    // P5 = end
    Point2D<double> P5(end.x(), end.y());
    // P4 = P5-1/5*te
    Point2D<double> P4 = P5-0.2*te;
    // P3 = 1/20*ae+2*P4-P5
    Point2D<double> P3 = 1/20.*ae+2*P4-P5;
    qDebug() << "Points are (x): " << P5.x << P4.x << P3.x << P2.x << P1.x << P0.x;
    qDebug() << "Points are (y): " << P5.y << P4.y << P3.y << P2.y << P1.y << P0.y;
    // coeff of polys:
    Point2D<double> a[6];
    // a5 = -P0+5P1-10P2+10P3-5P4+P5
    a[5] = (-1*P0)+(5*P1)-(10*P2)+(10*P3)-(5*P4)+P5;
    // a4 = 5P0-20P1+30P2-20P3+5P4
    a[4] = (5*P0)-(20*P1)+(30*P2)-(20*P3)+(5*P4);
    // a3 = -10P0+30P1-30P2+10P3
    a[3] = (-10*P0)+30*P1-30*P2+10*P3;
    // a2 = 10P0-20P1+10P2
    a[2] = 10*P0-20*P1+10*P2;
    // a1 = -5P0+5P1
    a[1] = -5*P0+5*P1;
    // a0 = P0
    a[0] = P0;
//    a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = Point2D<double>(10,10);
    qDebug() << "Polynomial coeffs are (x):" << a[5].x << a[4].x << a[3].x << a[2].x << a[1].x << a[0].x;
    qDebug() << "Polynomial coeffs are (y):" << a[5].y << a[4].y << a[3].y << a[2].y << a[1].y << a[0].y;
    // right now, just make u(t) = k*t
    // so that at u = 1, t = 3 sec
    // hence k = 1/3
    double k = 1/(2.);

    function<double(double)> xfunc = [=](double t)->double {
        double u = k*t;
        double upow = 1;
        double res = 0;
        for (int i = 0; i < 6; i++) {
            res += upow*a[i].x;
            upow *= u;
        }
        return res;
    };
    function<double(double)> yfunc = [=](double t)->double {
        double u = k*t;
        double res = 0;
        double upow = 1;
        for (int i = 0; i < 6; i++) {
            res += upow*a[i].y;
            upow *= u;
        }
        return res;
    };
    return Trajectory(xfunc, yfunc);

}

Trajectory ellipseGen(double x, double y, double a, double b, double startTheta, double f) {
    function<double(double)> xfunc = [=](double t)->double {
        return a*sin(2*PI*f*t + startTheta)+x;
    };
    function<double(double)> yfunc = [=](double t)->double {
        return b*cos(2*PI*f*t + startTheta)+y;
    };
    return Trajectory(xfunc, yfunc);
}
Trajectory cubic(Pose start, Pose end) {
    double k = 1 / 4.8;
    double d = sqrt((start.x() - end.x())*(start.x() - end.x()) + (start.y() - end.y())*(start.y() - end.y()));
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
    return Trajectory(xfunc, yfunc);
}
}
#endif // TRAJECTORYGENERATORS_HPP
