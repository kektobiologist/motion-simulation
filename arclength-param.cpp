#include "arclength-param.hpp"
#include <assert.h>
#include <time.h>
#include "alglib/interpolation.h"
#include "gsl/gsl_roots.h"
#include "gsl/gsl_errno.h"
#include "gsl/gsl_math.h"

namespace Integration {
gsl_integration_workspace * w = NULL;
double f(double u, void *integrand) {
  Integrand *in = (Integrand*) integrand;
  return (*in)(u);
}

//void func(double x, double xminusa, double bminusx, double &y, void *ptr)
//{
//    Integrand *in = (Integrand*)ptr;
//    y = (*in)(x);
//}

unsigned long long int rdtsc(void)
{
   unsigned long long int x;
   unsigned a, d;

   __asm__ volatile("rdtsc" : "=a" (a), "=d" (d));

   return ((unsigned long long)a) | (((unsigned long long)d) << 32);;
}

struct func_params
{
    double s;
    Spline& p;
};

double func(double u, void *params) {

    struct func_params *ps = (func_params *) params;
    return integrate(ps->p, 0, u)-ps->s;
}

double func_df(double u, void *params) {

    struct func_params *ps = (func_params *) params;

  Spline &pin = (Spline&) ps->p;
  return pin(u);
}

void func_fdf(double u, void *params, double *y, double *dy) {

    struct func_params *ps = (func_params *) params;
  Spline &pin = (Spline&) ps->p;
  *y = integrate(ps->p, 0, u)-ps->s;

  *dy = pin(u);
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

//double integrate_alglib(Integrand& p, double a, double b){

//    using namespace alglib;
//    autogkstate s;
//    autogkreport rep;
//    double v;

//    autogksmooth(a,b,s);
//    alglib::autogkintegrate(s, func, &p);
//    autogkresults(s,v,rep);

//    return v;
//    return 0;
//}


const gsl_root_fdfsolver_type *T;
gsl_root_fdfsolver *sf;

double optimise_u(Spline& p, double s, double e){

    int status;
      int iter = 0, max_iter = 100;

        double x0, x = s/e, r_expected=s;


    gsl_function_fdf F;
    F.f = &func;
    F.df = &func_df;
    F.fdf = &func_fdf;

    func_params fp = {s,p};
    F.params = &fp;

    T = gsl_root_fdfsolver_newton;
      sf = gsl_root_fdfsolver_alloc (T);
      gsl_root_fdfsolver_set (sf, &F, x);

      //printf("Using %s method\n", gsl_root_fdfsolver_name(sf));
      //printf("%-5s %10s %10s %10s\n", "iter", "root", "error", "err(est)");

      do
         {
           iter++;
           status = gsl_root_fdfsolver_iterate (sf);
           x0 = x;
           x = gsl_root_fdfsolver_root (sf);
           status = gsl_root_test_delta (x, x0, 0, 1e-3);

           if(status== GSL_SUCCESS)printf("Converged:\n");
           printf("%5d %10.7f %+10,7f %10.7f\n", iter, x, x-r_expected, x- x0);
         }
        while (status == GSL_CONTINUE && iter < max_iter);

        gsl_root_fdfsolver_free (sf);

       return x;
}

double getArcLengthParam(Spline& p, double s, double full) {
    // newton's method to find u for which arlength(p(0) to p(u)) = s;

  unsigned long long int t1 = rdtsc();
  unsigned long long int t2 = rdtsc();

  if (full < 0) {
    full = integrate(p, 0, 1);
  }

  //Using newton-rhapson technique in gsl
  double u = optimise_u(p,s,full);
//  assert(s >= 0);
//  //double u = s/full;  // initial guess;
//  double error = 1000;
  int iter = 0;
//  while (fabs(error) > 1e-3 && iter < 60) {
////      if (iter > 20)
////          qDebug() << "iter" << iter;
//    iter++;
//    error = integrate(p,0,u)-s;
//    u = u - error/p(u);
//  }
  // printf("iter = %d\n", iter);
  unsigned long long int t3 = rdtsc();
  unsigned long long int my_code_time = (t3-t2) - (t2-t1);

  if(iter>20 || my_code_time > 50000)
  qDebug() << "Time for optimisation, Iterations - " << my_code_time ;//<< " " << iter;

  return u;
}
}
