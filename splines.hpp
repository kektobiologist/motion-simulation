#ifndef SPLIENS_HPP
#define SPLIENS_HPP
#include "trajectory.hpp"
#include "alglib/interpolation.h"

// currently only able to handle starting and ending point
// need to extend so that it handles more points
class QuinticBezierSpline: public Spline {
protected:
  ParamPoly p;
public:
  QuinticBezierSpline(Pose start, Pose end, double vls, double vrs, double vle, double vre);
  virtual double x(double u) const;
  virtual double y(double u) const;
  virtual double xd(double u) const;
  virtual double yd(double u) const;
  virtual double xdd(double u) const;
  virtual double ydd(double u) const;
};

class CubicSpline: public Spline {
protected:
  alglib::spline1dinterpolant splineX, splineY;
public:
  // midPoints is an array of points through which the path should be interpolated.
  // NOTE: only x, y coordinates of midPoints are used, theta is ignored.
  CubicSpline(Pose start, Pose end, vector<Pose> midPoints = vector<Pose>());
  virtual double x(double u) const;
  virtual double y(double u) const;
  virtual double xd(double u) const;
  virtual double yd(double u) const;
  virtual double xdd(double u) const;
  virtual double ydd(double u) const;
  virtual double xddd(double u) const;
  virtual double yddd(double u) const;
  // returns max of ABSOLUTE value of curvature
  virtual double maxk(double *u_low = NULL) const;
};
#endif // SPLIENS_HPP

