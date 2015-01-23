#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include "pose.h"
#include <QString>
#include <deque>
using namespace std;
// Constants required by generateControl, directly copied from most recent version of kgpkubs.
const float MAX_BOT_LINEAR_VEL_CHANGE  = 4;
// NOTE(arpit): changed for real bots. for sim make this 100.0
const float MAX_BOT_SPEED              = 120.0;
const float MIN_BOT_SPEED              = 10.0;
const int BOT_POINT_THRESH             = 147;
const int CLEARANCE_PATH_PLANNER       = 400;
const int BOT_FINALVEL_THRESH          = 600;

const int timeLCMs = 16;
const double timeLC = timeLCMs*0.001;
const int NUMTICKS = 300;
#define SGN(x) (((x)>0)?1:(((x)<0)?(-1):0))
namespace Controllers {

MiscData kgpkubs(Pose initialPose, Pose finalPose, int &vl, int &vr, double prevSpeed, double finalSpeed = 0);
MiscData CMU(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double finalSpeed = 0);
MiscData PController(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double finalSpeed = 0);
MiscData PolarBased(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double finalSpeed = 0);
MiscData PolarBidirectional(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double finalSpeed = 0);
// functions for GA
void PolarBasedGA(Pose s, Pose e, int &vl, int &vr, double k1, double k2, double k3); // for use in GA
}
typedef MiscData(*FType)(Pose, Pose, int &, int &, double, double);
typedef std::pair<QString, FType> FPair;



class ControllerWrapper { // a wrapper to implement controller for a robot. Currently able to handle packet delay.
    FType fun;
    deque<pair<int,int> > uq; // controls queue. .first = vl, .second = vr
    double prevSpeed; //storing seperately since k = 0 means uq is empty
    int k;                    // the num of packet delay
public:
    ControllerWrapper(FType fun, int start_vl, int start_vr, int k):fun(fun), k(k) {
        for(int i = 0; i < k; i++)
            uq.push_back(make_pair<int,int>((int)start_vl,(int)start_vr));
        prevSpeed = 0;
    }
    Pose getPredictedPose(Pose s) {
        Pose x = s;
        for(deque<pair<int,int> >::iterator it = uq.begin(); it != uq.end(); it++) {
            x.updateNoDelay(it->first, it->second, timeLC);
        }
        return x;
    }
    MiscData genControls(Pose s, Pose e, int &vl, int &vr, double finalVel = 0) {
        Pose x = s;
        for(deque<pair<int,int> >::iterator it = uq.begin(); it != uq.end(); it++) {
            x.updateNoDelay(it->first, it->second, timeLC);
        }
        MiscData m = (*fun)(x, e, vl, vr, prevSpeed, finalVel);
        prevSpeed = max(fabs(vl), fabs(vr));
        uq.push_back(make_pair<int,int>((int)vl, (int)vr));
        uq.pop_front();
        return m;
    }
};
#endif // CONTROLLERS_H
