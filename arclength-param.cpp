#include "arclength-param.hpp"
#include <assert.h>
namespace Integration {
gsl_integration_workspace * w = NULL;
double f(double u, void *integrand) {
  Integrand *in = (Integrand*) integrand;
  return (*in)(u);
}

double integrate(Integrand& i, double s, double e) {
  double result, error;
  if (w == NULL) {
      w = gsl_integration_workspace_alloc (1000);
  }
  gsl_function F;
  F.function = f;
  F.params = &i;
  gsl_integration_qags (&F, s, e, 0, 1e-7, 1000,
                        w, &result, &error);

  return result;
}

double getArcLengthParam(Spline& p, double s, double full) {
    // newton's method to find u for which arlength(p(0) to p(u)) = s;
  if (full < 0) {
    full = integrate(p, 0, 1);
  }
  assert(s >= 0);
  double u = s/full;  // initial guess;
  double error = 1000;
  int iter = 0;
  while (fabs(error) > 1e-3 && iter < 60) {
      if (iter > 20)
          qDebug() << "iter" << iter;
    iter++;
    error = integrate(p,0,u)-s;
    u = u - error/p(u);
  }
  // printf("iter = %d\n", iter);
  return u;
}
}
