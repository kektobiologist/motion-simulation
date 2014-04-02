#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include "pose.h"
#include <QString>
// Constants required by generateControl, directly copied from most recent version of kgpkubs.
#define KD_ANGLE 0.1
const float MAX_BOT_LINEAR_VEL_CHANGE  = 3;
const float MAX_BOT_SPEED              = 80.0;
const int BOT_POINT_THRESH             = 147;
const int CLEARANCE_PATH_PLANNER       = 400;
const float DRIBBLER_BALL_ANGLE_RANGE  = 0.10f;

const int timeLCMs = 16;
const double timeLC = timeLCMs*0.001;
const int NUMTICKS = 300;
#define SGN(x) (((x)>0)?1:(((x)<0)?(-1):0))
namespace Controllers {
void turnToAngle(float turnAngleLeft,int &vl, int &vr );
void kgpkubs(Pose initialPose, Pose finalPose, int &vl, int &vr);
void CMU(Pose s, Pose e, int &vl, int &vr);
void PController(Pose s, Pose e, int &vl, int &vr);
void PolarBased(Pose s, Pose e, int &vl, int &vr);

// functions for GA
void PolarBasedGA(Pose s, Pose e, int &vl, int &vr, double k1, double k2, double k3); // for use in GA
}
typedef void(*FType)(Pose, Pose, int &, int &);
typedef std::pair<QString, FType> FPair;

#endif // CONTROLLERS_H
