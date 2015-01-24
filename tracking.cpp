#include "tracking.hpp"
#include <math.h>
#include <QDebug>
#include <assert.h>

const double Trajectory::deltaT = 0.001;
double sgn(double x) {
    return (x > 0) - (0 > x);
}

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

MiscData Tracker::genControls(Pose s, int &vl, int &vr, int prevVl, int prevVr, double t) {
    Q_UNUSED(prevVl);
    Q_UNUSED(prevVr);
    Pose ref(traj.x(t), traj.y(t), traj.theta(t));
    double ur1 = traj.v(t);
    double ur2 = traj.thetad(t);
    Error err(ref, s);
    double zeta = 0, omegan = 0, g = 0;
    double k1 = 2*zeta*omegan;
    double k2 = k1;
    double k3;
    if (fabs(ur1) < 1) {
        k3 = g*fabs(ur1);
    } else {
        k3 = (omegan*omegan - ur2*ur2)/fabs(ur1);
    }
    // NOTE: hardcoding k3 = 0!
    k3 = 0;
    // v = K.e
    double v1 = -k1*err.e1;
    double v2 = -sgn(ur1)*k2*err.e2 -k3*err.e3;
    double v = ur1*cos(err.e3) - v1;
    double w = ur2-v2;
    vl = v - Constants::d*w/2;
    vr = v + Constants::d*w/2;
    // vl, vr transform now
    return MiscData(ur1, ur2, v1, v2);
}
