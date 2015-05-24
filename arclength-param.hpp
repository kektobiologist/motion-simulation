#ifndef ARCLENGTHPARAM_HPP
#define ARCLENGTHPARAM_HPP
#include "trajectory.hpp"
#include <gsl/gsl_integration.h>

// functions for arc-length parametrizing a spline
namespace Integration {
extern gsl_integration_workspace * w;
double f(double u, void *integrand);
double integrate(Integrand& i, double s, double e);
// full = complete length of Spline from 0 to 1.
// iter: no. of iterations it takes to converge, useful for debugging/testing performance
double getArcLengthParam(Spline &p, double s, double full = -1, int *iter=0,int method=0);
vector<double> getInflectionPoints(Spline &p, double start_u, double end_u);
double s_formula(Spline &p, double b);
void computeBezierMatrices(Spline &p);
void refreshMatrix();
void computeInverseBezierMatrices(Spline &p);
void computeSplineApprox(Spline &p);
}
#endif // ARCLENGTHPARAM_HPP
