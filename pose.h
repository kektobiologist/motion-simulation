 #ifndef POSE_H
#define POSE_H

/* Pose defines the position and angle of the robot.
 * It is in REAL WORLD coordinates (m and radians).
 * NOTE: vl, vr are taken in ticks only. dt is in ms
 */

class Pose
{
    double x_, y_, theta_;
    static const double d          = 6.5; //distance between wheels in cm
    static const double ticksToCmS = 1.08; //still only approximate...
    static const double fieldXConvert = 29.75;
    static const double fieldYConvert = 27.33333;
public:
    double x() { return x_ * fieldXConvert;}                 // returns in strategy coordinate system
    double y() { return y_ * fieldYConvert;}                 // returns in strategy coordinate system
    double theta() { return theta_;}                         // already strategy coordinates
    Pose(double x, double y, double theta);                  // takes in Strategy coordinates!
    Pose();
    void update(int vl, int vr, double dt); // takes vl, vr in ticks. Implicitly converts to cm/s!! updates pose.
};

#endif // POSE_H
