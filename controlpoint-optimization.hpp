#ifndef CONTROLPOINTOPTIMIZATION_HPP
#define CONTROLPOINTOPTIMIZATION_HPP
#include <gsl/gsl_multimin.h>
#include "splines.hpp"
#include "pose.h"

namespace Optimization {
struct OptParams {
    Pose start, end;
    double vls, vrs, vle, vre;
    OptParams(): start(), end(), vls(0), vrs(0), vle(0), vre(0){}
    OptParams(Pose start, Pose end, double vls, double vrs, double vle, double vre): start(start),
        end(end), vls(vls), vrs(vrs), vle(vle), vre(vre) {}
};

// optimization function
double f_cubic2CP(const gsl_vector* x, void * params);
Trajectory *cubicSpline2CPOptimization(Pose start, Pose end, double vls, double vrs, double vle, double vre);
}
#endif // CONTROLPOINTOPTIMIZATION_HPP

