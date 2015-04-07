// helper functions useful for simulation. Actual simulation stuff is mostly still in pose.cpp
// TODO: move as much stuff from dialog.cpp as possible to here
#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include "controller-wrapper.hpp"
#include "vision-velocity.hpp"
#include <algorithm>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_multifit.h>
#include <QDebug>
#include "tracking.hpp"
struct RegData {
    double rho, gamma, delta, v_l, v_r;
    double timeMs;
    RegData():rho(0), gamma(0), delta(0), v_l(0), v_r(0), timeMs(0){}
    RegData(double rho, double gamma, double delta, double v_l, double v_r, double timeMs):rho(rho), gamma(gamma), delta(delta), v_l(v_l), v_r(v_r), timeMs(timeMs){}
};

class Simulator {
private:
    Pose poses[NUMTICKS];
    int vls[NUMTICKS], vrs[NUMTICKS];
    float vls_calc[NUMTICKS], vrs_calc[NUMTICKS];  // vl, vr reverse-calculated from vision data using VisionVelocity
    MiscData miscData[NUMTICKS];
    // functions for GA
    double fitnessFunction(double k1, double k2, double k3); // runs the PolarBasedGA function with k1, k2, k3 values.
public:
    Simulator() {}
    double simulate(Pose startPose, Pose endPose, FType func, int start_vl, int start_vr, bool isBatch = false);
                                                   // vls_calc[i] ~ vls[i] etc.
                                                   // implements delay control logic, for any given controller.
                                                   // (I removed the old simulate function that did not use wrapper)
    double simulate(Pose startPose, Trajectory* traj, int start_vl, int start_vr, bool isBatch = false);
    QString batchSimulation(FType fun);
    void regression(vector<RegData> func);
    int getVls(int idx) const{ return vls[idx];}
    int getVrs(int idx) const { return vrs[idx];}
    int getVls_calc(int idx) const { return vls_calc[idx];}
    int getVrs_calc(int idx) const { return vrs_calc[idx];}
    MiscData getMiscData(int idx) const {
        return miscData[idx];
    }
    Pose getPoses(int idx) const { return poses[idx];}

};
#endif // SIMULATION_HPP
