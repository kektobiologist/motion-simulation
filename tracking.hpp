#ifndef TRACKING_HPP
#define TRACKING_HPP
#include <functional>
#include "pose.h"
#include <sys/time.h>
#include "trajectory.hpp"
typedef double(*ParamFunc)(double);



// for tracking a trajectory.
// tracker should maintain its own time!(?) no, ControllerWrapper does it now.
// NOTE: error coordinates are in cm!
class Tracker {
    Trajectory *traj;
    struct Error {
        double e1, e2, e3;
        Error():e1(0), e2(0), e3(0) {}
        Error(Pose ref, Pose p) {
            Pose diff = ref - p;
            e1 = cos(p.theta())*diff.x() + sin(p.theta())*diff.y();
            e2 = -sin(p.theta())*diff.x() + cos(p.theta())*diff.y();
            e3 = diff.theta();
            e1 = e1/Constants::fieldXConvert;
            e2 = e2/Constants::fieldXConvert;
            // error coordinates are in cm!!
        }
    };
public:
    Tracker() {}
    Tracker(Trajectory *tr): traj(tr) {}
    void setTraj(Trajectory *tr) {
        traj = tr;
    }
    MiscData genControls(Pose s, int &vl, int &vr, int prevVl, int prevVr, double t);
};

#endif // TRACKING_HPP
