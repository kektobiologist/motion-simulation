#include "tracking.hpp"
#include <math.h>
#include <QDebug>
#include <assert.h>

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

MiscData Tracker::genControls(Pose s, int &vl, int &vr) {
    if (isFirstCall) {
        isFirstCall = false;
        gettimeofday(&startTime, 0);
    }
    struct timeval nowTime;
    gettimeofday(&nowTime, 0);
    float t = nowTime.tv_sec+nowTime.tv_usec/1000000.0;
    Pose ref(traj.x(t), traj.y(t), traj.theta(t));
    double ur1 = traj.v(t);
    double ur2 = traj.thetad(t);
    Error err(ref, s);
    double zeta = 1, omegan = 1000.0;
//    double k1 = k2 = 2*zeta*omegan;

}
