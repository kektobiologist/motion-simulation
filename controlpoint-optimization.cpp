#include "controlpoint-optimization.hpp"
#include "splines.hpp"
#include <vector>
using namespace std;
#include "drawable.h"
#include "collision-checking.h"
#include <QFile>
#include <QTextStream>
#include <QString>
#include <fstream>
#include <alglib/optimization.h>
#include "bayesopt/bayesopt/bayesopt.h"
#include "bayesopt/bayesopt/parameters.h"
#include <popot/rng_generators.h>
#include <iostream>
using namespace alglib;
typedef popot::rng::CRNG RNG_GENERATOR;
#include <popot/popot.h>
extern RenderArea *gRenderArea;
//double Optimization::f_cubic2CP(const gsl_vector *x, void *params_)
//{
//    OptParams *params = static_cast<OptParams*>(params_);
//    Pose cp1(gsl_vector_get(x,0)*fieldXConvert, gsl_vector_get(x,1)*fieldXConvert, 0);
//    Pose cp2(gsl_vector_get(x,2)*fieldXConvert, gsl_vector_get(x,3)*fieldXConvert, 0);
//    vector<Pose> midPoints;
//    midPoints.push_back(cp1);
//    midPoints.push_back(cp2);
//    CubicSpline *p = new CubicSpline(params->start, params->end, midPoints);
//    SplineTrajectory *st = new SplineTrajectory(p, params->vls, params->vrs, params->vle, params->vre);
//    double time = st->totalTime();
//    // use maxk also as a cost.
//    // try using ONLY maxk as cost function?
//    //double maxk = p->maxk();
////    delete st;
//    //vector<pair<double,float> > mp = p->lmaxk();
//    //p->lmaxk();
//    return time;
//    //return maxk;
//}


//Trajectory *Optimization::cubicSpline2CPOptimization(Pose start, Pose end, double vls, double vrs, double vle, double vre)
//{
//    OptParams params(start, end, vls, vrs, vle, vre);
//    const gsl_multimin_fminimizer_type *T =
//    gsl_multimin_fminimizer_nmsimplex2;
//    gsl_multimin_fminimizer *s = NULL;
//    gsl_vector *ss, *x;
//    gsl_multimin_function minex_func;

//    size_t iter = 0;
//    int status;
//    double size;

//    int n = 4;  // number of optimization params. here they are 2 CPs (x,y) in cm
//    /* Starting point */
//    // set some good starting values for the 2 CPs
////    Pose cp1((start.x()*2+end.x())*1/3., (start.y()*2+end.y())*1/3., 0);
////    Pose cp2((start.x()+2*end.x())*1/3., (start.y()+2*end.y())*1/3., 0);
//    Pose cp1(1000,1000,0);
//    Pose cp2(-1000,-1000,0);
//    x = gsl_vector_alloc (n);
//    gsl_vector_set(x, 0, cp1.x()/fieldXConvert);
//    gsl_vector_set(x, 1, cp1.y()/fieldXConvert);
//    gsl_vector_set(x, 2, cp2.x()/fieldXConvert);
//    gsl_vector_set(x, 3, cp2.y()/fieldXConvert);

//    /* Set initial step sizes to 100 */
//    ss = gsl_vector_alloc (n);
//    gsl_vector_set_all (ss, 100.0);

//    /* Initialize method and iterate */
//    minex_func.n = n;
//    minex_func.f = f_cubic2CP;
//    minex_func.params = &params;

//    s = gsl_multimin_fminimizer_alloc (T, n);
//    gsl_multimin_fminimizer_set (s, &minex_func, x, ss);

//    do
//    {
//      iter++;
//      status = gsl_multimin_fminimizer_iterate(s);

//      if (status)
//        break;

//      size = gsl_multimin_fminimizer_size (s);
//      status = gsl_multimin_test_size (size, 1e-2);

//      if (status == GSL_SUCCESS)
//        {
//          printf ("converged to minimum at\n");
//        }

//      printf ("%5d %10.3e %10.3e f() = %7.3f size = %.3f\n",
//              iter,
//              gsl_vector_get (s->x, 0),
//              gsl_vector_get (s->x, 1),
//              s->fval, size);
//    }
//    while (status == GSL_CONTINUE && iter < 100);

//    // make the trajectory now
//    SplineTrajectory *st;
//    {
//        Pose cp1(gsl_vector_get(s->x,0)*fieldXConvert, gsl_vector_get(s->x,1)*fieldXConvert, 0);
//        Pose cp2(gsl_vector_get(s->x,2)*fieldXConvert, gsl_vector_get(s->x,3)*fieldXConvert, 0);
//        static PointDrawable *pt1 = NULL, *pt2 = NULL;
//        if (pt1)
//            delete pt1;
//        if (pt2)
//            delete pt2;
//        pt1 = new PointDrawable(QPointF(cp1.x(), cp1.y()), gRenderArea);
//        pt2 = new PointDrawable(QPointF(cp2.x(), cp2.y()), gRenderArea);
//        vector<Pose> midPoints;
//        midPoints.push_back(cp1);
//        midPoints.push_back(cp2);
//        CubicSpline *p = new CubicSpline(start, end, midPoints);
//        st = new SplineTrajectory(p, vls, vrs, vle, vre);
////        double maxk_u, maxk;
////        maxk = p->maxk(&maxk_u);
////        qDebug() << "maxk = " << maxk << ", maxk_u = " << maxk_u;
//    }
//    gsl_vector_free(x);
//    gsl_vector_free(ss);
//    gsl_multimin_fminimizer_free (s);

//    return st;
//}


// make n cp optimizer

double Optimization::f_cubicnCP(const gsl_vector *x, void *params_)
{
    OptParams *params = static_cast<OptParams*>(params_);
    int n = params->n;
    std::vector<Pose> cps;
    for (int i = 0; i < n; i++) {
        cps.push_back(Pose(gsl_vector_get(x, 2*i)*fieldXConvert, gsl_vector_get(x, 2*i+1)*fieldXConvert, 0));
    }
    CubicSpline *p = new CubicSpline(params->start, params->end, cps);

    // check if collides with wall. if so, make the score = 1.5 x time score
    using CollisionChecking::LineSegment;
    vector<LineSegment> ls;
    ls.push_back(LineSegment(-HALF_FIELD_MAXX/fieldXConvert, -HALF_FIELD_MAXY/fieldXConvert, HALF_FIELD_MAXX/fieldXConvert, -HALF_FIELD_MAXY/fieldXConvert));
    ls.push_back(LineSegment(-HALF_FIELD_MAXX/fieldXConvert, HALF_FIELD_MAXY/fieldXConvert, HALF_FIELD_MAXX/fieldXConvert, HALF_FIELD_MAXY/fieldXConvert));
    ls.push_back(LineSegment(-HALF_FIELD_MAXX/fieldXConvert, -HALF_FIELD_MAXY/fieldXConvert, -HALF_FIELD_MAXX/fieldXConvert, +HALF_FIELD_MAXY/fieldXConvert));
    ls.push_back(LineSegment(HALF_FIELD_MAXX/fieldXConvert, -HALF_FIELD_MAXY/fieldXConvert, HALF_FIELD_MAXX/fieldXConvert, +HALF_FIELD_MAXY/fieldXConvert));
    bool collides_flag = false;

    vector<Pose> obs;
    obs.push_back(Pose(1000, 1000, 0));
    obs.push_back(Pose(-1000, -1000, 45));
    obs.push_back(Pose(1000, -1000, 90));
    obs.push_back(Pose(-1000, 1000, 135));

    // Check collision with line segments
    for (int i = 0; i < ls.size(); i++) {
        vector<Pose> collisions = CollisionChecking::cubicSpline_LineSegmentIntersection(*p, ls[i]);
        if (collisions.size()) {
            collides_flag = true;
            break;
        }
    }
    // Check collision with Obstacles
    qDebug() << "checking with obstacles " << endl;
                for (int i = 0 ; i < obs.size() ; i++) {
        if (CollisionChecking::cubicSpline_ObstacleCollision(*p, obs[i])){
            collides_flag = true;
            break;
        }
    }
    SplineTrajectory *st = new SplineTrajectory(p, params->vls, params->vrs, params->vle, params->vre);
    double time = st->totalTime();
    // use maxk also as a cost.
    // try using ONLY maxk as cost function?
    //double maxk = p->maxk();
//    delete st;
    //vector<pair<double,float> > mp = p->lmaxk();
    //p->lmaxk();
    if (collides_flag)
        time *= 3;

    return time;
    //return maxk;
}

Optimization::OptParams *gparams = NULL;
double Optimization::f_cubicnCP(const double *pts, size_t n1, size_t &count){//void *func_data){
    double *x = (double*) pts;
    OptParams *params = static_cast<OptParams*>(gparams);
    int n = (int)n1;
    std::vector<Pose> cps;
    for (int i = 0; i < n / 2; i++) {
//        qDebug() << x[2*i]*fieldXConvert << " " <<  x[2*i+1]*fieldXConvert << endl;
        cps.push_back(Pose(x[2*i]*fieldXConvert, x[2*i+1]*fieldXConvert, 0));
    }
    CubicSpline *p = new CubicSpline(params->start, params->end, cps);
    // check if collides with wall. if so, make the score = 1.5 x time score
    using CollisionChecking::LineSegment;
    vector<LineSegment> ls;
    ls.push_back(LineSegment(-HALF_FIELD_MAXX/fieldXConvert, -HALF_FIELD_MAXY/fieldXConvert, HALF_FIELD_MAXX/fieldXConvert, -HALF_FIELD_MAXY/fieldXConvert));
    ls.push_back(LineSegment(-HALF_FIELD_MAXX/fieldXConvert, HALF_FIELD_MAXY/fieldXConvert, HALF_FIELD_MAXX/fieldXConvert, HALF_FIELD_MAXY/fieldXConvert));
    ls.push_back(LineSegment(-HALF_FIELD_MAXX/fieldXConvert, -HALF_FIELD_MAXY/fieldXConvert, -HALF_FIELD_MAXX/fieldXConvert, +HALF_FIELD_MAXY/fieldXConvert));
    ls.push_back(LineSegment(HALF_FIELD_MAXX/fieldXConvert, -HALF_FIELD_MAXY/fieldXConvert, HALF_FIELD_MAXX/fieldXConvert, +HALF_FIELD_MAXY/fieldXConvert));
    bool collides_flag = false;
    for (int i = 0; i < ls.size(); i++) {
        vector<Pose> collisions = CollisionChecking::cubicSpline_LineSegmentIntersection(*p, ls[i]);
        if (collisions.size()) {
            collides_flag = true;
            break;
        }
    }
    count++;
    vector<Pose> obs;
    obs.push_back(Pose(1000, 1000, 0));
    obs.push_back(Pose(-800, -700, 45));
    obs.push_back(Pose(650, -700, 90));
    obs.push_back(Pose(-450, 300, 135));

    double min_dist = 1000000.0;
    for (int i = 0 ; i < obs.size() ; i++) {
        double dist = CollisionChecking::cubicSpline_ObstacleCollision(*p, obs[i]);
        if (dist < min_dist)
            min_dist = dist;
//            qDebug() << "here" << endl;
//            collides_flag = true;
//            break;
    }

    SplineTrajectory *st = new SplineTrajectory(p, params->vls, params->vrs, params->vle, params->vre);
    double time = st->totalTime();
//    qDebug() << min_dist << "bckjswerdv"<< 6 * BOT_RADIUS << endl;

//    else {
        if (min_dist < 1.5 * BOT_RADIUS){
            qDebug() << "old time" << time;
            time = time * (2.5 - (min_dist / (1.5 * BOT_RADIUS)));
            qDebug() << "new time" << time << endl;
        }
        else{
            if (collides_flag){
                time *= 10;
            }
    }
        std::ofstream outfile;

          outfile.open("logs/pso.txt", std::ios_base::app);
          outfile << count << " " << time << endl;
          outfile.close();

//        qDebug() << count << " " << time << endl;
    return time;
}

bool stop(double f, int epoch)
 {
   return (f <= 1.0) || (epoch >= 100);
 }

Trajectory *Optimization::cubicSplinenCPOptimization(Pose start, Pose end, double vls, double vrs, double vle, double vre, int n, std::string fileid)
{

    assert(n >= 0 && n <= 5);
    OptParams params(start, end, vls, vrs, vle, vre, n);
    gparams = &params;
     // ###########################################################################
//    /* Starting point */
//    // set some values for the control points
//    double cps[4] = {0};
//    for (int i = 0; i < n; i++) {
//        double x = rand()/(double)RAND_MAX*1000.*((rand()%2)*2-1);
//        double y = rand()/(double)RAND_MAX*1000.*((rand()%2)*2-1);
//        cps[2*i] = x/fieldXConvert; cps[2*i+1] = y/fieldXConvert;
//    }

//    QString filename = "/home/abhinav/Desktop/pathplanner_extras/log/" + QString::fromStdString(fileid) + ".log";
////    QFile file(filename);
////    file.open(QIODevice::WriteOnly| QIODevice::Text);
////    QTextStream stream(&file);

////    double x[4] = {};
//    const double bndl[4] = {-1000,-1000,-1000,-1000};
//    const double bndu[4] = {1000,1000,1000,1000};
//    double minf=0;
//    double epsg = 0.0000000001;
//    double epsf = 0;
//    double epsx = 0;
//    ae_int_t maxits = 0;
//    minlmstate state;
//    minlmreport rep;

////    minlmcreatev(2, x, 0.0001, state);
////    minlmsetbc(state, bndl, bndu);
////    minlmsetcond(state, epsg, epsf, epsx, maxits);
////    alglib::minlmoptimize(state, f_cubicnCP);
////    minlmresults(state, x, rep);

////    qDebug("in this %d\n", float(rep.terminationtype)); // EXPECTED: 4
////    qDebug("in this %s\n", x.tostring(4).c_str()); // EXPECTED: [-1,+1]

//    bopt_params bparams = initialize_parameters_to_default();
//    bparams.n_iterations = 100;
//    bparams.verbose_level = 4;
//    set_log_file(&bparams, filename.toStdString().c_str());
//    set_learning(&bparams,"L_MCMC");
//    int k= bayes_optimization(4,f_cubicnCP,NULL,bndl, bndu, cps, &minf, bparams);
//    stream << minf;
    // make the trajectory now
    // ##########################################################################
    RNG_GENERATOR::rng_srand();

     size_t colony_size = 10;
     size_t dimension = 4;
     size_t count=0;

     auto algo = popot::algorithm::abc(colony_size, dimension,
                       [] (size_t index) -> double { return -100; },
                       [] (size_t index) -> double { return  100; },
                       stop,
                       [&count,dimension] (double * params) -> double { return f_cubicnCP(params, dimension, count);});

     algo.init();
     algo.run();
     double * cps = new double[4];
     for (size_t i = 0 ; i < 4 ; i++) {
         cps[i] = algo.getBest()[i] * fieldXConvert;
         qDebug() << "cps" << cps[i] << endl;
     }
     std::cout << "Best minimum found :" << algo.getBest() << " in " << algo.getEpoch() << " steps " << std::endl;
     std::cout << "Position of the optimum : " << algo.getBest() << std::endl;
     std::cout << std::endl;
     std::cout << (int)count << " Function Evaluations" << std::endl;


    SplineTrajectory *st;
    {
        vector<Pose> cpsf;
        for (int i = 0; i < n; i++) {
            cpsf.push_back(Pose(cps[2*i], cps[2*i+1], 0));
        }
        static vector<PointDrawable*> pts;
        for (int i = 0; i < pts.size(); i++) {
            if (pts[i])
                delete pts[i];
        }
        pts.clear();
        for (int i = 0; i < n; i++) {
            PointDrawable *pt = new PointDrawable(QPointF(cpsf[i].x(), cpsf[i].y()), gRenderArea);
            pts.push_back(pt);
        }
        CubicSpline *p = new CubicSpline(start, end, cpsf);
        st = new SplineTrajectory(p, vls, vrs, vle, vre);
    }

    return st;
}
