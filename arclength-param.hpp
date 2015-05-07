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
double getArcLengthParam(Spline &p, double s, double full = -1);
vector<double> getInflectionPoints(Spline &p, double start_u, double end_u);
double s_formula(Spline &p, double b);
void computeBezierMatrices(Spline &p);
void refreshMatrix();
}
#endif // ARCLENGTHPARAM_HPP
