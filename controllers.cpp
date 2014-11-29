#include "controllers.h"
#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include <utility>
using namespace std;
namespace Controllers {

MiscData kgpkubs(Pose initialPose, Pose finalPose, int &vl, int &vr, double prevSpeed, double finalSpeed)
{
    Q_UNUSED(prevSpeed);
    int clearance = CLEARANCE_PATH_PLANNER;
    static float prevVel = 0;
    Vector2D<int> initial(initialPose.x(), initialPose.y());
    Vector2D<int> final(finalPose.x(), finalPose.y());
    double curSlope = initialPose.theta();
    double finalSlope = finalPose.theta();
    double theta = normalizeAngle(Vector2D<int>::angle(final, initial));
    double phiStar = finalSlope;
    double phi = curSlope;
    //directionalAngle = (cos(atan2(final.y - initial.y, final.x - initial.x) - curSlope) >= 0) ? curSlope : normalizeAngle(curSlope - PI);
    int dist = Vector2D<int>::dist(final, initial);  // Distance of next waypoint from the bot
    double alpha = normalizeAngle(theta - phiStar);
    double beta = (fabs(alpha) < fabs(atan2(clearance, dist))) ? (alpha) : SGN(alpha) * atan2(clearance, dist);
    double thetaD = normalizeAngle(theta + beta);
    float delta = normalizeAngle(thetaD - phi);
    double r = (sin(delta) * sin(delta)) * SGN(tan(delta));
    double t = (cos(delta) * cos(delta)) * SGN(cos(delta));
    if(!(t >= -1.0 && t <= 1.0 && r >= -1.0 && r <= 1.0)) {
     printf("what? delta = %f, initial = (%lf, %lf, %lf)\n", delta, initialPose.x(), initialPose.y(), initialPose.theta());
     assert(0);
    }
    float fTheta = asin(sqrt(fabs(r)));
    fTheta = 1 - fTheta/(PI/2);
    fTheta = pow(fTheta,2.2) ;
    float fDistance = (dist > BOT_POINT_THRESH*3) ? 1 : dist / ((float) BOT_POINT_THRESH *3);
    float fTot = fDistance * fTheta;
    fTot = 0.2 + fTot*(1-0.2);
    float profileFactor = MAX_BOT_SPEED * fTot;
    if(fabs(r)<0.11)
      profileFactor*=2;
    {
      if(fabs(profileFactor-prevVel)>MAX_BOT_LINEAR_VEL_CHANGE)
      {
        if(profileFactor>prevVel)
            profileFactor=prevVel+MAX_BOT_LINEAR_VEL_CHANGE;
        else
            profileFactor=prevVel-MAX_BOT_LINEAR_VEL_CHANGE;
      }
      prevVel=profileFactor;
    }
    if(profileFactor>1.5*MAX_BOT_SPEED)
      profileFactor = 1.5*MAX_BOT_SPEED;
    else if(profileFactor <-1.5*MAX_BOT_SPEED)
      profileFactor = -1.5*MAX_BOT_SPEED;
    prevVel=profileFactor;
    r *= 0.5*profileFactor;
    t *= profileFactor;

    vl = t-r;
    vr = t+r;
    return MiscData();
}

MiscData CMU(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double finalSpeed)
{
    Q_UNUSED(prevSpeed);
    int maxDis = 2*sqrt(HALF_FIELD_MAXX*HALF_FIELD_MAXX+HALF_FIELD_MAXY*HALF_FIELD_MAXY);
    Vector2D<int> initial(s.x(), s.y());
    Vector2D<int> final(e.x(), e.y());
    int distance = Vector2D<int>::dist(initial, final);
    double phi = s.theta();
    double theta = Vector2D<int>::angle(final, initial);
    double phiStar = e.theta();
    double alpha = normalizeAngle(theta-phiStar);
    double beta = atan2((maxDis/15.0), distance);
    double thetaD = theta + min(alpha, beta);
    thetaD = theta; // removing the phiStar part. wasnt able to get it to work right.
    double delta = normalizeAngle(thetaD - phi);
    double t = cos(delta)*cos(delta)*SGN(cos(delta));
    double r = sin(delta)*sin(delta)*SGN(sin(delta));
    vl = 100*(t-r);
    vr = 100*(t+r);
    return MiscData();
}

MiscData PController(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double finalSpeed)
{
    Q_UNUSED(prevSpeed);
    Vector2D<int> initial(s.x(), s.y());
    Vector2D<int> final(e.x(), e.y());
    int distance = Vector2D<int>::dist(initial, final);
    double angleError = normalizeAngle(s.theta() - Vector2D<int>::angle(final, initial));
    double v = 0;
    int maxDis = 2*sqrt(HALF_FIELD_MAXX*HALF_FIELD_MAXX+HALF_FIELD_MAXY*HALF_FIELD_MAXY);
    if(angleError > PI/2) {
        v = 0;
    } else {
        if(distance > maxDis/2) {
            v = MAX_BOT_SPEED;
        } else {
            v = (distance/(double)maxDis)*90+10;
        }
    }
    double w = -1.5*angleError;
    v *= Pose::ticksToCmS;
    vl = v - Pose::d*w/2;
    vr = v + Pose::d*w/2;
    if(abs(vl) > MAX_BOT_SPEED || abs(vr) > MAX_BOT_SPEED) {
        double max = abs(vl)>abs(vr)?abs(vl):abs(vr);
        vl = vl*MAX_BOT_SPEED/max;
        vr = vr*MAX_BOT_SPEED/max;
    }
    return MiscData();
}

/*  Aicardi M, Casalino G, Bicchi A, Balestrino A 1995 Closed loop steering of
unicycle-like vehicles via Lyapunov techniques. IEEE Robotics & Automation
Magazine 2(1):27–35
*/
MiscData PolarBased(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double finalSpeed)
{
    // NOTE: its preferable to call x(), y(), and theta() of each object exactly once since they may return different
    // values on each call.
    Vector2D<int> initial(s.x()-e.x(), s.y()-e.y());
    double etheta = e.theta();
    double theta = normalizeAngle(s.theta() - etheta);
    // rotate initial by -e.theta degrees;
    double newx = initial.x * cos(-etheta) - initial.y * sin(-etheta);
    double newy = initial.x * sin(-etheta) + initial.y * cos(-etheta);
    initial = Vector2D<int>(newx, newy);
    double rho = sqrt(initial.x*initial.x + initial.y*initial.y);
    double gamma = normalizeAngle(atan2(initial.y, initial.x) - theta + PI);
    double delta = normalizeAngle(gamma + theta);
    double k1 = 0.05, k2 = 4, k3 = 20;
    double v = k1*rho*cos(gamma);
    double w;
    if(gamma == 0) {
        w = k2*gamma+k1*cos(gamma)*(gamma+k3*delta);
    } else {
        w = k2*gamma+k1*sin(gamma)*cos(gamma)/gamma*(gamma + k3*delta);
    }
    // velocity profiling based on curvature
    double k = w/v; // k = curvature
    // scale curvature by 50.
    k *= 20;
    double lambda = 2;
    double beta = 0.7;
    double v_curve = MAX_BOT_SPEED/(1+beta*pow(fabs(k),lambda));
    if (v_curve < MIN_BOT_SPEED)
        v_curve = MIN_BOT_SPEED;    
    v *= Pose::ticksToCmS;
    vl = v - Pose::d*w/2;
    vr = v + Pose::d*w/2;
    double timeMs = 0.250*rho + 14.0 * sqrt(rho) + 100.0 * fabs(gamma);
    double speed = timeMs/timeLCMs<(prevSpeed/MAX_BOT_LINEAR_VEL_CHANGE)?prevSpeed-MAX_BOT_LINEAR_VEL_CHANGE:prevSpeed+MAX_BOT_LINEAR_VEL_CHANGE;    
    // use vcurve as the velocity
    // NOTE: adding vcurve and finalVel code
    // critical condition: if bot close to final point, v_curve = MAX_BOT_SPEED
    if (rho < BOT_POINT_THRESH && finalSpeed > MIN_BOT_SPEED) {
        v_curve = MAX_BOT_SPEED;
        vl = vr; // ? or vl = vr?
    }
    double rangeMin = max(prevSpeed - MAX_BOT_LINEAR_VEL_CHANGE, 0.0);
    double rangeMax = min(prevSpeed + MAX_BOT_LINEAR_VEL_CHANGE, v_curve);
    if (v_curve < prevSpeed - MAX_BOT_LINEAR_VEL_CHANGE) {
        rangeMin = rangeMax = v_curve;
    }
    // only consider finalSpeed when d < BOT_FINALVEL_THRESH
    if (rho <= BOT_FINALVEL_THRESH)
        speed = finalSpeed < rangeMax? max(rangeMin, finalSpeed) : min(rangeMax, finalSpeed);
    else
        speed = rangeMax;
    // critical condition : finalVel = 0, but bot not close to final pose
    if (rho > BOT_POINT_THRESH && speed < MIN_BOT_SPEED)
        speed = MIN_BOT_SPEED;

    // end
    if(speed > MAX_BOT_SPEED)
        speed = MAX_BOT_SPEED;
    else if (speed < 0)
        speed = 0;
    double max = fabs(vl)>fabs(vr)?fabs(vl):fabs(vr);
    if(max > 0) {
        vl = vl*speed/max;
        vr = vr*speed/max;
    }
    return MiscData(k, v_curve, finalSpeed, rangeMin, rangeMax);
}
MiscData PolarBidirectional(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double finalSpeed)
{
    MiscData m;
    static bool wasInverted = false; // keeps track of whether in the previous call, the bot was inverted or not.
    m = PolarBased(s, e, vl, vr, prevSpeed, finalSpeed);
    double v = (vl+vr)/2.0;
    wasInverted = false;
    if(v < 0 || (v == 0 && wasInverted) ) {
        s.setTheta(normalizeAngle(s.theta()+PI));
        m = PolarBased(s, e, vl, vr, prevSpeed, finalSpeed);
        swap(vl, vr);
        vl = -vl;
        vr = -vr;
        wasInverted = true;
    }
    return m;
}

void PolarBasedGA(Pose s, Pose e, int &vl, int &vr, double k1, double k2, double k3) // this function is old now, do not use.
{
    Vector2D<int> initial(s.x()-e.x(), s.y()-e.y());
    double theta = normalizeAngle(s.theta() - e.theta());
    // rotate initial by -e.theta degrees;
    double newx = initial.x * cos(-e.theta()) - initial.y * sin(-e.theta());
    double newy = initial.x * sin(-e.theta()) + initial.y * cos(-e.theta());
    initial = Vector2D<int>(newx, newy);
    double rho = sqrt(initial.x*initial.x + initial.y*initial.y);
    double gamma = normalizeAngle(atan2(initial.y, initial.x) - theta + PI);
    double delta = normalizeAngle(gamma + theta);
    double v = k1*rho*cos(gamma);
    double w;
    if(gamma == 0) {
        w = k2*gamma+k1*cos(gamma)*(gamma+k3*delta);
    } else {
        w = k2*gamma+k1*sin(gamma)*cos(gamma)/gamma*(gamma + k3*delta);
    }
    v *= Pose::ticksToCmS;
    vl = v - Pose::d*w/2;
    vr = v + Pose::d*w/2;
    double timeMs = 23 * sqrt(rho); // empirical
    double speed = timeMs<timeLCMs*5?timeMs/timeLCMs*(80/5):80;
    double max = fabs(vl)>fabs(vr)?fabs(vl):fabs(vr);
    if(max > 0) {
        vl = vl*speed/max;
        vr = vr*speed/max;
    }
}

}
