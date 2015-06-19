#include "trajectory-drawing.hpp"
#include "controller-wrapper.hpp"
namespace TrajectoryDrawing {
// what's wrong with this function? :(
void insertArcInPath(QPainterPath &p, double currentAngle, int vl, int vr, double dt) {  // NOTE: dt is in seconds!
    double x = p.currentPosition().x();
    double y = p.currentPosition().y();
    // NOTE: copying(!) all the code frome pose.cpp for finding the radius of curvature etc.
    double v = (vl+vr)/2.0;

    if(vl == vr) //special case handled seperately
    {
        p.lineTo(x + v * cos(currentAngle) * dt, y + v * sin(currentAngle) * dt);
        return;
    }

    double rho = Constants::d/2 * Constants::fieldXConvert *(vr+vl)/(vr-vl); //NOTE: assuming fieldXConvert ~ fieldYConvert!!
    double w = (vr-vl)/(Constants::d * Constants::fieldXConvert);

    double ICCx = x - rho * sin(currentAngle);
    double ICCy = y + rho * cos(currentAngle);

    double startAngleDeg = (currentAngle - PI/2)*180.0/PI;
    double sweepAngleDeg = w * dt * 180.0/PI;

    p.arcTo(ICCx-rho, ICCy-rho, 2*rho, 2*rho, startAngleDeg, sweepAngleDeg);
}
QPainterPath getTrajectoryPath(FType func, Pose s, int vl_s, int vr_s, Pose e, int vl_e, int vr_e,
                                          double timespanMs, double timeLCMs) {
    // using some code from simulate() in dialog.cpp
    QPainterPath p;
    p.moveTo(s.queryX(), s.queryY());
    double final_vel = max(fabs(vl_e), fabs(vr_e));
    ControllerWrapper dc(func, vl_s, vr_s, 0);  // not using any delay for trajectory drawing.
    Pose curPose = s;
    for (int i = 0; i*timeLCMs < timespanMs; i++) {
        int vl, vr;
        dc.genControls(curPose, e, vl, vr, final_vel);
        // patch up code;
//        insertArcInPath(p, curPose.theta(), vl, vr, timeLCMs*0.001);
        curPose.update(vl, vr, timeLCMs*0.001);
        p.lineTo(curPose.queryX(), curPose.queryY());
    }
    return p;
}
QPainterPath getTrajectoryPath(const Trajectory& traj, double timespanMs, double timeLCMs) {
    QPainterPath p;
    qDebug() << "Printing the x and y coordinates: \n" << traj.x(0)*fieldXConvert << " " << traj.y(0)*fieldXConvert;
    p.moveTo(traj.x(0)*fieldXConvert, traj.y(0)*fieldXConvert);
    for (int i = 1; i*timeLCMs < timespanMs; i++) {
        //qDebug() << traj.x(i*timeLCMs*0.001)*fieldXConvert << " " << traj.y(i*timeLCMs*0.001)*fieldXConvert;
        p.lineTo(traj.x(i*timeLCMs*0.001)*fieldXConvert, traj.y(i*timeLCMs*0.001)*fieldXConvert);
    }
    return p;
}
}
