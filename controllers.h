// definitions of some point to point controllers. trajectory generators are in trajectory-generators.hpp
#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include "pose.h"
#include <QString>
#include <deque>
#include "tracking.hpp"
#include <assert.h>
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



#endif // CONTROLLERS_H
