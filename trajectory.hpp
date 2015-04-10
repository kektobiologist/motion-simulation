#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
#include <vector>
#include <functional>
#include "pose.h"
#include <sys/time.h>
#include "velocity-profile.hpp"
using namespace std;
// should take time as seconds.
// NOTE: FOR SOME REASON, x, y, theta, thetad, v have to be in STRATEGY COORDINATES. I'll fix this later.
class Trajectory {
    std::function<double(double)> x_;
    std::function<double(double)> y_;
    static const double deltaT;
    virtual double xd(double t) const;
    virtual double yd(double t) const;
    virtual double xdd(double t) const;
    virtual double ydd(double t) const;
public:
    Trajectory() {
        x_ = [](double)->double{ return 0;};
        y_ = [](double)->double{ return 0;};
    }
    Trajectory(std::function<double(double)>& x_, std::function<double(double)>& y_): x_(x_), y_(y_) {}
    virtual double x(double t) const;
    virtual double y(double t) const;
    virtual double theta(double t) const;
    virtual double thetad(double t) const;
    virtual double v(double t) const;
};

class Integrand {
public:
  Integrand() {}
  virtual double operator()(double u) const = 0;
};

class ParamPoly: public Integrand {
protected:
  vector<double> ax, ay;
  int n;  // ax.size() = ay.size() = n
public:
  ParamPoly();
  ParamPoly(const vector<double> &ax, const vector<double> &ay);
  virtual double x(double u) const;
  virtual double y(double u) const;
  virtual double xd(double u) const;
  virtual double yd(double u) const;
  virtual double xdd(double u) const;
  virtual double ydd(double u) const;
  virtual double qd(double u) const; // = sqrt(xd*xd+yd*yd)
  virtual double operator()(double u) const; // returns qd(u);
};

// this is an interface class
// extend spline interface when needed
// eg.: addPoint, changePoint, remo
class Spline: public Integrand {
public:
    Spline();
    virtual double x(double u) const = 0;
    virtual double y(double u) const = 0;
    virtual double xd(double u) const = 0;
    virtual double yd(double u) const = 0;
    virtual double xdd(double u) const = 0;
    virtual double ydd(double u) const = 0;
    virtual double qd(double u) const;
    virtual double operator()(double u) const; // returns qd(u);
    virtual double k(double u) const;
};



// NOTE: x(t) and y(t) MUST be in cm!!!!
// NOTE: always create SplineTrajectory object with a dynamically allocated Spline.
// This class takes ownership of the passed Spline and is responsible for its destruction.
class SplineTrajectory: public Trajectory {
protected:
    Spline *p;
    vector<VelocityProfiling::ProfileDatapoint> profile;
    mutable double tPrev;  // so that multiple calculations are not done when x(t), y(t), theta(t) etc. called with same t.
    mutable double x_, y_, theta_, thetad_, v_; // these are returned, but calculated only in calculateAll(t);
    double full;
    void calculateAll(double t) const;
public:
    ~SplineTrajectory();
    SplineTrajectory(Spline *p, double vls, double vrs, double vle, double vre);
    virtual double x(double t) const;
    virtual double y(double t) const;
    virtual double theta(double t) const;
    virtual double thetad(double t) const;
    virtual double v(double t) const;
    virtual double totalTime() const;
};

#endif // TRAJECTORY_HPP

