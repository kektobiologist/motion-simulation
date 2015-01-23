#ifndef TRACKING_HPP
#define TRACKING_HPP
#include <functional>
#include "pose.h"
#include <sys/time.h>
typedef double(*ParamFunc)(double);

// should take time as seconds.
class Trajectory {
    std::function<double(double)> x_;
    std::function<double(double)> y_;
    static const double deltaT;
public:
    Trajectory() {
        x_ = [](double)->double{ return 0;};
        y_ = [](double)->double{ return 0;};
    }
    Trajectory(std::function<double(double)>& x_, std::function<double(double)>& y_): x_(x_), y_(y_) {}
    double x(double t) const;
    double y(double t) const;
    double xd(double t) const;
    double yd(double t) const;
    double xdd(double t) const;
    double ydd(double t) const;
    double theta(double t) const;
    double thetad(double t) const;
    double v(double t) const;
};

// for tracking a trajectory.
// tracker should maintain its own time!(?)
class Tracker {
    Trajectory traj;
    int prevVl, prevVr;
    struct Error {
        double e1, e2, e3;
        Error():e1(0), e2(0), e3(0) {}
        Error(Pose ref, Pose p) {
            Pose diff = ref - p;
            e1 = cos(p.theta())*diff.x() + sin(p.theta())*diff.y();
            e2 = -sin(p.theta())*diff.x() + cos(p.theta())*diff.y();
            e3 = diff.theta();
        }
    };
    struct timeval startTime;
    bool isFirstCall;
public:
    Tracker(): traj(), prevVl(0), prevVr(0), startTime(), isFirstCall(true) {}
    void reset() {  // set the clock to zero.
        isFirstCall = true;
        gettimeofday(&startTime, NULL);
    }
    void setTraj(Trajectory tr) {
        traj = tr;
    }
    MiscData genControls(Pose s, int &vl, int &vr);
};

#endif // TRACKING_HPP
