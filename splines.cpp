#include "splines.hpp"
#include "alglib/interpolation.h"
QuinticBezierSpline::QuinticBezierSpline(Pose start, Pose end, double vls, double vrs, double vle, double vre)
{
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
        as = Point2D<double>(0,0);
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
        ae = Point2D<double>(0,0);
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
    P0 = P0/fieldXConvert;
    P1 = P1/fieldXConvert;
    P2 = P2/fieldXConvert;
    P3 = P3/fieldXConvert;
    P4 = P4/fieldXConvert;
    P5 = P5/fieldXConvert;

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
    vector<double> ax, ay;
    for (int i = 0; i < 6; i++) {
        ax.push_back(a[i].x);
        ay.push_back(a[i].y);
    }
    p = ParamPoly(ax, ay);
}

double QuinticBezierSpline::x(double u) const
{
    return p.x(u);
}

double QuinticBezierSpline::y(double u) const
{
    return p.y(u);
}

double QuinticBezierSpline::xd(double u) const
{
    return p.xd(u);
}

double QuinticBezierSpline::yd(double u) const
{
    return p.yd(u);
}

double QuinticBezierSpline::xdd(double u) const
{
    return p.xdd(u);
}

double QuinticBezierSpline::ydd(double u) const
{
    return p.ydd(u);
}



CubicSpline::CubicSpline(Pose start, Pose end, std::vector<Pose> midPoints)
{
    double d = sqrt((start.x() - end.x())*(start.x() - end.x()) + (start.y() - end.y())*(start.y() - end.y()));
    d = d/fieldXConvert;
    double x1 = start.x()/fieldXConvert;
    double x2 = end.x()/fieldXConvert;
    double y1 = start.y()/fieldXConvert;
    double y2 = end.y()/fieldXConvert;
    double th1 = start.theta();
    double th2 = end.theta();
    {
        using namespace alglib;
        double n = midPoints.size()+2; // number of points to interpolate on
        vector<double> x(n,0), y(n,0), u(n,0);
        x[0] = x1;
        x[n-1] = x2;
        y[0] = y1;
        y[n-1] = y2;
        for (int i = 0; i < n; i++) {
            u[i] = i/(double)(n-1);
        }
        for (int i = 1; i < n-1; i++ ) {
            x[i] = midPoints[i-1].x()/fieldXConvert;
            y[i] = midPoints[i-1].y()/fieldXConvert;
        }
        alglib::real_1d_array AU, AY, AX;
        AU.setcontent(u.size(), &(u[0]));
        AY.setcontent(y.size(), &(y[0]));
        AX.setcontent(x.size(), &(x[0]));
        spline1dbuildcubic(AU, AX, u.size(), 1, d*cos(th1), 1, d*cos(th2), splineX);
        spline1dbuildcubic(AU, AY, u.size(), 1, d*sin(th1), 1, d*sin(th2), splineY);
    }
//    vector<double> a(4,0), b(4,0);
//    a[3] = d * cos(th2) + d * cos(th1) - 2 * (x2 - x1);
//    a[2] = 3 * (x2 - x1) - d * cos(th2) - 2 * d * cos(th1);
//    a[1] = d * cos(th1);
//    a[0] = x1;
//    b[3] = d * sin(th2) + d * sin(th1) - 2 * (y2 - y1);
//    b[2] = 3 * (y2 - y1) - d * sin(th2) - 2 * d * sin(th1);
//    b[1] = d * sin(th1);
//    b[0] = y1;
//    // make them cm from strategy coordinates
//    for (int i = 0; i < 4; i++) {
//        a[i] = a[i]/Constants::fieldXConvert;
//        b[i] = b[i]/Constants::fieldXConvert;
//    }
//    p = ParamPoly(a, b);

}

double CubicSpline::x(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineX, u, s, ds, d2s);
    return s;
}

double CubicSpline::y(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineY, u, s, ds, d2s);
    return s;
}

double CubicSpline::xd(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineX, u, s, ds, d2s);
    return ds;
}

double CubicSpline::yd(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineY, u, s, ds, d2s);
    return ds;
}

double CubicSpline::xdd(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineX, u, s, ds, d2s);
    return d2s;
}

double CubicSpline::ydd(double u) const
{
    double s, ds, d2s;
    alglib::spline1ddiff(splineY, u, s, ds, d2s);
    return d2s;
}

double CubicSpline::maxk(double *u) const
{
    using namespace alglib;
    real_2d_array tblx, tbly;
    int nx, ny;
    alglib::spline1dunpack(splineX, nx, tblx);
    alglib::spline1dunpack(splineY, ny, tbly);
    assert (nx == ny);
    double maxk = 0;
    double maxk_u = 0;
    // iterate through each segment, find the maxk
    qDebug() << "new call:";
    for (int i = 0; i < nx-1; i++) {
        double u_low = tblx[i][0], u_high = tblx[i][1];
        assert(tbly[i][0] == u_low && tbly[i][1] == u_high);
        // get coefficients
        double a[4], b[4]; // a = x coeff, b = y coeff
        for (int j = 0; j < 4; j++) {
            a[j] = tblx[i][j+2];
            b[j] = tbly[i][j+2];
        }
        qDebug() << "coeff (x), (y) =  " << a[3] << a[2] << a[1] << a[0] <<
                    b[3] << b[2] << b[1] << b[0] << "ulow, uhigh=" << u_low << u_high;
        // get k value at beginning
        if (fabs(this->k(u_low)) > maxk) {
            maxk = fabs(this->k(u_low));
            maxk_u = u_low;
        }
        // get k value at end of this spline component
        if (fabs(this->k(u_high)) > maxk) {
            maxk = fabs(this->k(u_high));
            maxk_u = u_high;
        }
        // get u value for which xd*ydd-yd*xdd is extremum
        // this doesn't actually find extrema of k, but i think it should be good enough
        // solution is:
        // u = (b1*a3-a1*b3)/(2*(a2*b3-b2*a3))
        if (a[2]*b[3]-b[2]*a[3] != 0) {
            // the spline in alglib takes input t = u-u_low
            double t_ext = (b[1]*a[3]-a[1]*b[3])/2./(a[2]*b[3]-b[2]*a[3]);
            if (t_ext >= 0 && t_ext <= u_high-u_low) {
                double u_ext = t_ext + u_low;
                if (fabs(this->k(u_ext)) > maxk) {
                    maxk = fabs(this->k(u_ext));
                    maxk_u = u_ext;
                }
            }
        }
    }
//    qDebug() << "maxk_u = " << maxk_u << ", maxk = " << maxk;
    if (u)
        *u = maxk_u;
    return maxk;
}
