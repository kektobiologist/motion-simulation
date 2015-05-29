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
    qDebug() << "Hello!kk\n";
    return (xd(u)*ydd(u)-yd(u)*xdd(u))/pow(xd(u)*xd(u)+yd(u)*yd(u), 1.5);
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
        u = Integration::getArcLengthParam(*p,s,full);
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
        u = Integration::getArcLengthParam(*p, s, full);
    }
    x_ = p->x(u);
    y_ = p->y(u);
    theta_ = atan2(p->yd(u), p->xd(u));
    v_ = profile[i].v + dt*at;
    thetad_ = p->k(u)*v_;
//    qDebug() << "computed: t = " << t << "u = " << u << ", x = " << x_ << "y = " << y_
    //             << ", theta=" << theta_ << "thetad = " << thetad_ << "v = " << v_;
}

SplineTrajectory::~SplineTrajectory()
{
    if (p)
        delete p;
}

SplineTrajectory::SplineTrajectory(Spline *p, double vls, double vrs, double vle, double vre): p(p)
{
    profile = VelocityProfiling::generateVelocityProfile(*p, 1000, vls, vrs, vle, vre);
    full = Integration::integrate(*p, 0, 1);
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

double SplineTrajectory::totalTime() const
{
    return profile[profile.size()-1].t;
}
