#include "pose.h"
#include <math.h>
#include <algorithm>
/* Constructor takes in x, y in Strategy coordinates! */
Pose::Pose(double x, double y, double theta):
    x_(x/fieldXConvert),
    y_(y/fieldYConvert),
    theta_(theta)
{    
    for(int i = 0; i < numPacketDelay; i++) {
        vlq.push(0); vrq.push(0);
    }
}

Pose::Pose()
{
    x_ = 0; y_ = 0; theta_ = 0;
    for(int i = 0; i < numPacketDelay; i++) {
        vlq.push(0); vrq.push(0);
    }
}

void Pose::update(int vl_ticks, int vr_ticks, double dt)
{
    update_2(vl_ticks, vr_ticks, dt);
}

void Pose::updateNoDelay(int vl, int vr, double dt)
{
    update_1(vl, vr, dt);
}

void Pose::update_1(int vl_ticks, int vr_ticks, double dt)
{
    double vl = vl_ticks * ticksToCmS;
    double vr = vr_ticks * ticksToCmS;

    if(vl_ticks == vr_ticks) //special case handled seperately
    {
        double v = (vl+vr)/2;
        x_ += v * cos(theta_) * dt;
        y_ += v * sin(theta_) * dt;
        return;
    }

    double rho = d/2*(vr+vl)/(vr-vl);
    double w = (vr-vl)/d;

    x_ += rho*(sin(theta_ + w * dt) - sin(theta_));
    y_ += -rho*(cos(theta_ + w * dt) - cos(theta_));
    theta_ += w * dt;
    while(theta_ > PI) theta_ -= PI*2;
    while(theta_ <= -PI) theta_ += PI*2;
}

void Pose::update_2(int vl_ticks, int vr_ticks, double dt)
{
    vlq.push(vl_ticks);
    vrq.push(vr_ticks);
    int vl = vlq.front();
    int vr = vrq.front();
    vlq.pop(); vrq.pop();
    update_1(vl, vr, dt);
}

double Pose::x() const
{
    double withError =  x_ * fieldXConvert + randStdNormal()*HALF_FIELD_MAXX*xUncertainty/100.0;
    return withError;
}

double Pose::y() const
{
    double withError =  y_ * fieldYConvert + randStdNormal()*HALF_FIELD_MAXY*yUncertainty/100.0;
    return withError;
}

double Pose::theta() const
{
    double withError = normalizeAngle(theta_ + thetaUncertainty*randStdNormal()*PI/100);
    return withError;
}
