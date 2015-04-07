#ifndef VELOCITYPROFILE_HPP
#define VELOCITYPROFILE_HPP
#include <stdio.h>
#include <gsl/gsl_integration.h>
#include <vector>
#include <functional>
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include "constants.h"
using namespace std;
class Spline;
namespace VelocityProfiling {
// isolated constraints

// k: curvature ( = w/v), 1/cm
// vsat: max vl (or vr) in  cm/s
// vwmax: max centripetal acceleration, cm/s^2
// return: vmax, cm/s
double vmax_isolated(double k, double vsat = Constants::vsat, double vwmax = Constants::vwmax );

typedef pair<double, double> Interval;
// acceleration constraints

// translational acceleration constraint
// atmax: max translational acceleration, +ve, cm/s^2
// dels: distance between this and old point, cm
Interval trans_acc_limits(double vold, double atmax, double dels);

// rotational acceleration constraint
// awmax: maximum rotational acceleration, 1/s^2
vector<Interval> rot_acc_limits(double vold, double kold, double k, double dels,
  double awmax);

// ellipse constraint
// will add later
struct ProfileDatapoint {
    double u, v, s, t; // u = param, v = velocity (cm/s), s = arclength from 0 to u, t = time take from 0 to u
    ProfileDatapoint();
    ProfileDatapoint(double u, double v, double s, double t);
    bool operator<(const ProfileDatapoint &dp) const;
};


// velocity profile
// p: spline
// numPoints: number of planning points to set velocity for
vector<ProfileDatapoint> generateVelocityProfile(Spline &p, int numPoints, double vls, double vrs, double vle, double vre);
}
#endif // VELOCITYPROFILE_HPP

