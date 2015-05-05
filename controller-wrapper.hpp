#ifndef CONTROLLERWRAPPER_HPP
#define CONTROLLERWRAPPER_HPP
#include "controllers.h"
class ControllerWrapper { // a wrapper to implement controller for a robot (both point to point and tracking control
                          //. Currently able to handle packet delay.
    // this is for point to point control
    FType fun;
    // this is common stuff
    deque<pair<int,int> > uq; // controls queue. .first = vl, .second = vr
    double prevVl, prevVr; //storing seperately since k = 0 means uq is empty
    int k;                    // the num of packet delay
    enum {POINTCTRL, TRACKCTRL} ctrlType;
    // this is stuff for tracking controller
    Tracker tracker;
    struct timeval startTime;
    bool isFirstCall;
    MiscData genControls_(Pose s, Pose e, int &vl, int &vr, double finalVel = 0);
    MiscData genControls_(Pose s, int &vl, int &vr, double time = 0, bool useTime = false);
public:
    ControllerWrapper(FType fun, int start_vl, int start_vr, int k);
    ControllerWrapper(Trajectory *traj, int start_vl, int start_vr,  int k);
    void reset();
    void setTraj(Trajectory *traj);
    Pose getPredictedPose(Pose s);
    double getCurrentTimeS() const; // when tracking, get the current time the tracker is working on.
    MiscData genControls(Pose s, Pose e, int &vl, int &vr, double finalVel = 0);
    MiscData genControlsTrajSim(Pose s, int &vl, int &vr, double t);

};
#endif // CONTROLLERWRAPPER_HPP
