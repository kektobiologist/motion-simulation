#ifndef CONTROLPOINTOPTIMIZATION_HPP
#define CONTROLPOINTOPTIMIZATION_HPP
#include <gsl/gsl_multimin.h>
#include "splines.hpp"
#include "pose.h"
#include <alglib/optimization.h>
using namespace alglib;
namespace Optimization {
struct OptParams {
    Pose start, end;
    double vls, vrs, vle, vre;
    // n = number of control points
    int n;
    OptParams(): start(), end(), vls(0), vrs(0), vle(0), vre(0), n(0){}
    OptParams(Pose start, Pose end, double vls, double vrs, double vle, double vre, int n): start(start),
        end(end), vls(vls), vrs(vrs), vle(vle), vre(vre), n(n) {}
};

// optimization function
double f_cubicnCP(const gsl_vector* x, void * params);
double f_cubicnCP(unsigned int n, const double *x, double *gradient, void *func_data);
Trajectory *cubicSplinenCPOptimization(Pose start, Pose end, double vls, double vrs, double vle, double vre, int n, std::string fileid);
}
#endif // CONTROLPOINTOPTIMIZATION_HPP

