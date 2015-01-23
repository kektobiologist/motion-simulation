#include "vision-velocity.hpp"
#include "geometry.h"
#include "pose.h"
#include <assert.h>

namespace VisionVelocity {

void strategyToRealConversion(BotPose &p) {
    // converts from strategy to actual coordinates (cm)
    p.x /= Constants::fieldXConvert;
    p.y /= Constants::fieldYConvert;
}

void calcBotVelocity(BotPose p1, BotPose p2, float timeMs, float &vl, float &vr) {
    strategyToRealConversion(p1);
    strategyToRealConversion(p2);

    double delX = p2.x - p1.x;
    double delY = p2.y - p1.y;
    double delTheta = normalizeAngle(p2.theta - p1.theta); // assuming |delTheta| < PI, which is safe to assume
                                                           // for ~ 16 ms of rotation at any speed (even 120,-120?).
    assert(timeMs > 0);
    // w * timeMs = delTheta
    double w = delTheta / (timeMs * 0.001);
    if (delTheta < 1e-2 && delTheta > -1e-2) {  // be realistic
        // bot should be headed straight, but again confusion
        // so taking projection of (delX, delY) along (cos(theta), sin(theta)) as displacement.
        double dispLength = delX*cos(p1.theta) + delY*sin(p1.theta);
        vl = dispLength / (timeMs * 0.001);
        vl = vl / Constants::ticksToCmS;
        vr = vl;
        return;
    }
    // we calculate 2 rho's, based on delX and delY, and take average
    double rho1 = delX / (sin(p2.theta) - sin(p1.theta));
    double rho2 = -delY / (cos(p2.theta) - cos(p1.theta));
    // try harmonic mean?
//    double rho = (rho1 + rho2) / 2;
    double rho = 2*rho1*rho2 / (rho1 + rho2);
    vl = w * (rho - Constants::d/2.0) / Constants::ticksToCmS;
    vr = w * (rho + Constants::d/2.0) / Constants::ticksToCmS;

    // use the translational, rotational system
//    double v_trans = sqrt(delX*delX + delY*delY) / (timeMs * 0.001);
//    double v_rot = w;
//    vl = v_trans - Constants::d*v_rot/2;
//    vr = v_trans + Constants::d*v_rot/2;
//    vl /= Constants::ticksToCmS;
//    vr /= Constants::ticksToCmS;
    return;
}

void calcBallVelocity(int delx, int dely, float timeMs, float &vx, float &vy) {
    BotPose p(delx, dely, 0);
    vx = p.x / (timeMs * 0.001);
    vy = p.y / (timeMs * 0.001);
    return;
}
}
