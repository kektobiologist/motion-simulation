#include "trajectory.hpp"
#include <assert.h>
#include "arclength-param.hpp"
const double Trajectory::deltaT = 0.001;


double Trajectory::x (double t) const{
    return x_(t);
}

double Trajectory::y(double t) const {
    return y_(t);
}

double Trajectory::xd(double t)const {
    return (x(t+deltaT) - x(t))/deltaT;
}

double Trajectory::yd(double t) const{
    return (y(t+deltaT) - y(t))/deltaT;
}

double Trajectory::xdd(double t) const{
    return (xd(t+deltaT) - xd(t))/deltaT;
}

double Trajectory::ydd(double t) const{
    return (yd(t+deltaT) - yd(t))/deltaT;
}

double Trajectory::theta(double t) const{
    return atan2(yd(t), xd(t)); // +{0,1}pi for direction
}

double Trajectory::thetad(double t) const{
    double xd_ = xd(t);
    double yd_ = yd(t);
    if (xd_ == 0 && yd_ == 0) {
        qDebug() << "omega can't be calculated for trajectory!";
        assert(0);
    }
    return (xd_*ydd(t) - yd_*xdd(t))/(xd_*xd_ + yd_*yd_);
}

double Trajectory::v(double t) const{
    double xd_ = xd(t);
    double yd_ = yd(t);
    return sqrt(xd_*xd_ + yd_*yd_);
}

double ParamPoly::x(double u) const {
  double w = 1;
  double ans = 0;
  for (int i = 0; i < n; i++) {
    ans += w*ax[i];
    w *= u;
  }
  return ans;
}

double ParamPoly::y(double u) const {
  double w = 1;
  double ans = 0;
  for (int i = 0; i < n; i++) {
    ans += w*ay[i];
    w *= u;
  }
  return ans;
}

double ParamPoly::xd(double u) const {
  double w = 1;
  double ans = 0;
  for (int i = 1; i < n; i++) {
    ans += w*ax[i]*i;
    w *= u;
  }
  return ans;
}

double ParamPoly::yd(double u) const {
  double w = 1;
  double ans = 0;
  for (int i = 1; i < n; i++) {
    ans += w*ay[i]*i;
    w *= u;
  }
  return ans;
}

double ParamPoly::xdd(double u) const
{
    double w = 1;
    double ans = 0;
    for (int i = 2; i < n; i++) {
      ans += w*ax[i]*i*(i-1);
      w *= u;
    }
    return ans;
}

double ParamPoly::ydd(double u) const
{
    double w = 1;
    double ans = 0;
    for (int i = 2; i < n; i++) {
      ans += w*ay[i]*i*(i-1);
      w *= u;
    }
    return ans;
}

double ParamPoly::qd(double u) const {
  double xd_ = xd(u), yd_ = yd(u);
  return sqrt(xd_*xd_+yd_*yd_);
}

double ParamPoly::operator()(double u) const {
  return qd(u);
}

ParamPoly::ParamPoly(const vector<double> &ax, const vector<double> &ay): ax(ax), ay(ay)
{
    assert(ax.size() == ay.size());
    n = ax.size();
}

ParamPoly::ParamPoly(): n(0) {
}

Spline::Spline() {
}

double Spline::qd(double u) const
{
    double xd_ = xd(u), yd_ = yd(u);
    return sqrt(xd_*xd_+yd_*yd_);
}

double Spline::operator()(double u) const
{
    return qd(u);
}

double Spline::k(double u) const
{
    return (xd(u)*ydd(u)-yd(u)*xdd(u))/pow(xd(u)*xd(u)+yd(u)*yd(u), 1.5);
}




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

using VelocityProfiling::ProfileDatapoint;
void SplineTrajectory::calculateAll(double t) const
{
    assert(t >= 0);
    tPrev = t;
    vector<ProfileDatapoint>::const_iterator upper = upper_bound(profile.begin(), profile.end(), ProfileDatapoint(0.,0.,0.,t));
    double dt, s, u, at;
    int i;
    if (upper == profile.end()) {
        i = profile.size()-1;
        // handle
        dt = t-profile[i].t;
        s = profile[i].s + profile[i].v*dt;
        u = Integration::getArcLengthParam(p,s,full);
        at = 0;
    } else {
        assert(upper != profile.begin());// upper can't possibly be v.begin() since v[0].t = 0.. ?
        i = upper-profile.begin()-1;
        // we are now in the interval between i and i+1
        at = (profile[i+1].v-profile[i].v)/(profile[i+1].t-profile[i].t);
        dt = t - profile[i].t;
        assert(dt>=0);
        s = 0.5*at*dt*dt+profile[i].v*dt+profile[i].s;
        assert(s <= full);
        // now find u
        u = Integration::getArcLengthParam(p, s, full);
    }
    x_ = p.x(u);
    y_ = p.y(u);
    theta_ = atan2(p.yd(u), p.xd(u));
    v_ = profile[i].v + dt*at;
    thetad_ = p.k(u)*v_;
//    qDebug() << "computed: t = " << t << "u = " << u << ", x = " << x_ << "y = " << y_
//             << ", theta=" << theta_ << "thetad = " << thetad_ << "v = " << v_;
}

SplineTrajectory::SplineTrajectory(Spline &p, double vls, double vrs, double vle, double vre): p(p)
{
    profile = VelocityProfiling::generateVelocityProfile(p, 100, vls, vrs, vle, vre);
    full = Integration::integrate(p, 0, 1);
    tPrev = -1;
}

double SplineTrajectory::x(double t) const
{
    if (t-tPrev >1e-8 || t-tPrev <-1e-8) {
        calculateAll(t);
    }
    return x_;
}

double SplineTrajectory::y(double t) const
{
    if (t-tPrev >1e-8 || t-tPrev <-1e-8) {
        calculateAll(t);
    }
    return y_;
}

double SplineTrajectory::theta(double t) const
{
    if (t-tPrev >1e-8 || t-tPrev <-1e-8) {
        calculateAll(t);
    }
    return theta_;
}

double SplineTrajectory::thetad(double t) const
{
    if (t-tPrev >1e-8 || t-tPrev <-1e-8) {
        calculateAll(t);
    }
    return thetad_;
}

double SplineTrajectory::v(double t) const
{
    if (t-tPrev >1e-8 || t-tPrev <-1e-8) {
        calculateAll(t);
    }
    return v_;
}
