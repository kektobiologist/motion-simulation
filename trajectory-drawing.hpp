#ifndef TRAJECTORYDRAWING_HPP
#define TRAJECTORYDRAWING_HPP
#include <vector>
#include <algorithm>
#include <utility>
#include "controllers.h"
#include "pose.h"
#include "tracking.hpp"
#include <QPainterPath>

using namespace std;
namespace TrajectoryDrawing {


QPainterPath getTrajectoryPath(FType func, Pose s, int vl_s, int vr_s, Pose e, int vl_e, int vr_e,
                                          double timespanMs, double timeLCMs);
QPainterPath getTrajectoryPath(const Trajectory& traj, double timespanMs, double timeLCMs);
}
#endif // TRAJECTORYDRAWING_HPP
