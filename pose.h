#ifndef POSE_H
#define POSE_H
#include <queue>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include "geometry.h"
#include "constants.h"
/* Pose defines the position and angle of the robot.
 * It is in REAL WORLD coordinates (m and radians).
 * NOTE: vl, vr are taken in ticks only. dt is in ms
 */
class Pose
{
    double x_, y_, theta_;
public:
    static const double d          = 6.5; //distance between wheels in cm
    static const double ticksToCmS = 1.08; //still only approximate...
    static const double fieldXConvert = 29.75;
    static const double fieldYConvert = 27.33333;
    // NOTE(arpit): Uncertainties should be non-zero when simulating. Currently 0 since bot data is fetched from vision.
    static const double xUncertainty = 0;//0.5; // Uncertainty is in %age of max value. eg. 1% where fabs(x) <= 1000 means fabs(error) <= 10
    static const double yUncertainty = 0;//0.5;
    static const double thetaUncertainty = 0;//3;
private:
    void update_1(int vl_ticks, int vr_ticks, double dt); // simple update, without delay.
    void update_2(int vl_ticks, int vr_ticks, double dt); // delay of 1 tick bw updates.
    std::queue<int> vlq, vrq;      // q to implement packet delay.
public:
    // NOTE(arpit): numPacketDelay and update() specified here is only used in simulation.
    static const int numPacketDelay = 3; // num of packets to delay in update
    double randStdNormal() {double x = rand()/(double)RAND_MAX; return sqrt(-2*log(x))*cos(2*3.14159265359*x);} // returns random number from std normal distribution
    double x(); // returns in strategy coordinate system
    double y(); // returns in strategy coordinate system
    double theta(); // already in strategy coordinates
    double queryX() const { return x_ * fieldXConvert;}   // These do not
    double queryY() const { return y_ * fieldYConvert;}   // apply gaussian error
    double queryTheta() const { return theta_;}           // .
    Pose(double x, double y, double theta);    // takes in Strategy coordinates!
    Pose();
    void update(int vl, int vr, double dt);               // takes vl, vr in ticks. Implicitly converts to cm/s!! updates pose.
    void updateNoDelay(int vl, int vr, double dt);        // updates without any delay mechanics.
    void setTheta(double newtheta) {theta_ = newtheta;}   // will add setX and setY if/when needed
};

inline double dist(Pose p1, Pose p2) {
    return sqrt((double)(p1.x()-p2.x())*(p1.x()-p2.x()) + (p1.y()-p2.y())*(p1.y()-p2.y()));
}

#endif // POSE_H
