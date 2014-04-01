#ifndef POSE_H
#define POSE_H
#include <queue>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include "geometry.h"
/* Pose defines the position and angle of the robot.
 * It is in REAL WORLD coordinates (m and radians).
 * NOTE: vl, vr are taken in ticks only. dt is in ms
 */
const int HALF_FIELD_MAXX              = 2975;
const int HALF_FIELD_MAXY              = 2050;
class Pose
{
    double x_, y_, theta_;
public:
    static const double d          = 6.5; //distance between wheels in cm
    static const double ticksToCmS = 1.08; //still only approximate...
    static const double fieldXConvert = 29.75;
    static const double fieldYConvert = 27.33333;
private:
    void update_1(int vl_ticks, int vr_ticks, double dt); // simple update, without delay.
    void update_2(int vl_ticks, int vr_ticks, double dt); // delay of 1 tick bw updates.
    std::queue<int> vlq, vrq;      // q to implement packet delay.
public:
    static const int numPacketDelay = 3; // num of packets to delay in update
    double randStdNormal() {double x = rand()/(double)RAND_MAX; return sqrt(-2*log(x))*cos(2*3.14159265359*x);} // returns random number from std normal distribution
    double x() { return x_ * fieldXConvert + randStdNormal()*HALF_FIELD_MAXX*1/1000.0;} // returns in strategy coordinate system
    double y() { return y_ * fieldYConvert + randStdNormal()*HALF_FIELD_MAXY*1/1000.0;} // returns in strategy coordinate system
    double theta() { return normalizeAngle(theta_ + 10*randStdNormal()*PI/1000);}       // already strategy coordinates
    Pose(double x, double y, double theta);                                             // takes in Strategy coordinates!
    Pose();
    void update(int vl, int vr, double dt);               // takes vl, vr in ticks. Implicitly converts to cm/s!! updates pose.
    void updateNoDelay(int vl, int vr, double dt);        // updates without any delay mechanics.
    void setTheta(double newtheta) {theta_ = newtheta;}   // will add setX and setY if/when needed
};

inline double dist(Pose p1, Pose p2) {
    return sqrt((double)(p1.x()-p2.x())*(p1.x()-p2.x()) + (p1.y()-p2.y())*(p1.y()-p2.y()));
}

#endif // POSE_H
