#include "tracking.hpp"
#include <math.h>
#include <QDebug>
#include <assert.h>

double sgn(double x) {
    return (x > 0) - (0 > x);
}

MiscData Tracker::genControls(Pose s, int &vl, int &vr, int prevVl, int prevVr, double t) {
    Q_UNUSED(prevVl);
    Q_UNUSED(prevVr);
    Pose ref(traj->x(t)*fieldXConvert, traj->y(t)*fieldXConvert, traj->theta(t));
    double ur1 = traj->v(t);
    double ur2 = traj->thetad(t);
    // err coordinates are in cm!
    Error err(ref, s);
    double zeta = 1, omegan = 10, g = .1;
    double k1 = 2*zeta*omegan;
    double k3 = k1;
    double k2;
    if (fabs(ur1) < 1) {
        k2 = g*fabs(ur1);
    } else {
        k2 = (omegan*omegan - ur2*ur2)/fabs(ur1);
    }
    // NOTE: hardcoding k3 = 0!
//    k3 = 0;
    // v = K.e
//    err.e3 = 0;
    double v1 = -k1*err.e1;
    double v2 = -sgn(ur1)*k2*err.e2 -k3*err.e3;
    double v = ur1*cos(err.e3) - v1;
    double w = ur2-v2;
    vl = (v - Constants::d*w/2)/Constants::ticksToCmS;
    vr = (v + Constants::d*w/2)/Constants::ticksToCmS;
    // vl, vr transform now
//    if (vl > 120)
//        vl = 120;
//    else if (vl < -120)
//        vl = -120;
//    if (vr > 120)
//        vr = 120;
//    else if (vr < -120)
//        vr = -120;
    return MiscData(ur1, ur2, v1, v2, t, v);
}
