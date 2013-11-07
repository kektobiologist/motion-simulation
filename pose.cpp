#include "pose.h"
#include <math.h>
/* Constructor takes in x, y in Strategy coordinates! */
Pose::Pose(double x, double y, double theta):
    x_(x/fieldXConvert),
    y_(y/fieldYConvert),
    theta_(theta)
{    
}

Pose::Pose()
{
    x_ = 0; y_ = 0; theta_ = 0;
}

void Pose::update(int vl_ticks, int vr_ticks, double dt)
{
    static const double PI = 3.14159265359;
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
    double w = 2*(vr-vl)/d;

    x_ += rho*(sin(theta_ + w * dt) - sin(theta_));
    y_ += -rho*(cos(theta_ + w * dt) - cos(theta_));
    theta_ += w * dt;
    while(theta_ > PI) theta_ -= PI*2;
    while(theta_ <= -PI) theta_ += PI*2;
}
