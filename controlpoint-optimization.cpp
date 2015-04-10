#include "controlpoint-optimization.hpp"
#include "splines.hpp"
#include <vector>
using namespace std;
double Optimization::f_cubic2CP(const gsl_vector *x, void *params_)
{
    OptParams *params = static_cast<OptParams*>(params_);
    Pose cp1(gsl_vector_get(x,0)*fieldXConvert, gsl_vector_get(x,1)*fieldXConvert, 0);
    Pose cp2(gsl_vector_get(x,2)*fieldXConvert, gsl_vector_get(x,3)*fieldXConvert, 0);
    vector<Pose> midPoints;
    midPoints.push_back(cp1);
    midPoints.push_back(cp2);
    CubicSpline *p = new CubicSpline(params->start, params->end, midPoints);
    SplineTrajectory *st = new SplineTrajectory(p, params->vls, params->vrs, params->vle, params->vre);
    double time = st->totalTime();
    delete st;
    return time;
}


Trajectory *Optimization::cubicSpline2CPOptimization(Pose start, Pose end, double vls, double vrs, double vle, double vre)
{

    OptParams params(start, end, vls, vrs, vle, vre);
    const gsl_multimin_fminimizer_type *T =
    gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = NULL;
    gsl_vector *ss, *x;
    gsl_multimin_function minex_func;

    size_t iter = 0;
    int status;
    double size;

    int n = 4;  // number of optimization params. here they are 2 CPs (x,y) in cm
    /* Starting point */
    // set some good starting values for the 2 CPs
    Pose cp1((start.x()*2+end.x())*1/3., (start.y()*2+end.y())*1/3., 0);
    Pose cp2((start.x()+2*end.x())*1/3., (start.y()+2*end.y())*1/3., 0);
//    Pose cp1(1000,1000,0);
//    Pose cp2(-1000,-1000,0);
    x = gsl_vector_alloc (n);
    gsl_vector_set(x, 0, cp1.x()/fieldXConvert);
    gsl_vector_set(x, 1, cp1.y()/fieldXConvert);
    gsl_vector_set(x, 2, cp2.x()/fieldXConvert);
    gsl_vector_set(x, 3, cp2.y()/fieldXConvert);

    /* Set initial step sizes to 30 */
    ss = gsl_vector_alloc (n);
    gsl_vector_set_all (ss, 30.0);

    /* Initialize method and iterate */
    minex_func.n = n;
    minex_func.f = f_cubic2CP;
    minex_func.params = &params;

    s = gsl_multimin_fminimizer_alloc (T, n);
    gsl_multimin_fminimizer_set (s, &minex_func, x, ss);

    do
    {
      iter++;
      status = gsl_multimin_fminimizer_iterate(s);

      if (status)
        break;

      size = gsl_multimin_fminimizer_size (s);
      status = gsl_multimin_test_size (size, 1e-2);

      if (status == GSL_SUCCESS)
        {
          printf ("converged to minimum at\n");
        }

      printf ("%5d %10.3e %10.3e f() = %7.3f size = %.3f\n",
              iter,
              gsl_vector_get (s->x, 0),
              gsl_vector_get (s->x, 1),
              s->fval, size);
    }
    while (status == GSL_CONTINUE && iter < 100);

    // make the trajectory now
    SplineTrajectory *st;
    {
        Pose cp1(gsl_vector_get(s->x,0)*fieldXConvert, gsl_vector_get(s->x,1)*fieldXConvert, 0);
        Pose cp2(gsl_vector_get(s->x,2)*fieldXConvert, gsl_vector_get(s->x,3)*fieldXConvert, 0);
        vector<Pose> midPoints;
        midPoints.push_back(cp1);
        midPoints.push_back(cp2);
        CubicSpline *p = new CubicSpline(start, end, midPoints);
        st = new SplineTrajectory(p, vls, vrs, vle, vre);
    }
    gsl_vector_free(x);
    gsl_vector_free(ss);
    gsl_multimin_fminimizer_free (s);

    return st;
}
