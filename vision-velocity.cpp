#include "vision-velocity.hpp"
#include "geometry.h"

namespace VisionVelocity {
void calcBotVelocity(BotPose p1, BotPose p2, float timeMs, float &vl, float &vr) {
    double delX = p2.x - p1.x;
    double delY = p2.y - p1.y;
    double delTheta = normalizeAngle(p2.theta - p1.theta); // assuming |delTheta| < PI, which is safe to assume
                                                           // for ~ 16 ms of rotation at any speed.
//    if (delThta)

}
}
