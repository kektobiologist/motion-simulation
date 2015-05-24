#ifndef TESTS_HPP
#define TESTS_HPP
#include "pose.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <vector>
#include "splines.hpp"
#include "arclength-param.hpp"
using namespace std;
// ONLY include in dialog.cpp
namespace Tests {
Pose randomPose() {
    double x = (rand()/(double)RAND_MAX)*HALF_FIELD_MAXX;
    x *= 2*(rand()%2)-1;
    double y = (rand()/(double)RAND_MAX)*HALF_FIELD_MAXY;
    y *= 2*(rand()%2)-1;
    double theta = normalizeAngle((rand()/(double)RAND_MAX)*M_PI*2.);
    return Pose(x, y, theta);
}

CubicSpline randomCubicSpline(int nCP) {
    // create random start/ end pose
    Pose start = randomPose(), end = randomPose();
    // add nCP random midPoints;
    vector<Pose> midPoints;
    for (int i = 0; i < nCP; i++) {
        midPoints.push_back(randomPose());
    }
    return CubicSpline(start, end, midPoints);
}

void arclengthParam_test(int nTests) {
    // create a random spline
    CubicSpline p = randomCubicSpline(2);
    Integration::refreshMatrix();
    //Integration::computeInverseBezierMatrices(p);
    Integration::computeSplineApprox(p);
    // call arclengthParam on some u values
    double full = Integration::integrate(p, 0, 1);
    int maxIter, maxIterSP, maxIterB;
    maxIter=maxIterSP=maxIterB=0;
    double avgIter, avgIterSP, avgIterB = 0;
    for (int i = 0; i < nTests; i++) {
        int iter;
        double s = (full*i)/nTests;
        double u = Integration::getArcLengthParam(p, s, full, &iter,2);
        if (iter > maxIter)
            maxIter = iter;
        avgIter += iter;

        u = Integration::getArcLengthParam(p, s, full, &iter,0);
        if (iter > maxIterSP)
            maxIterSP = iter;
        avgIterSP += iter;

        u = Integration::getArcLengthParam(p, s, full, &iter,1);
        if (iter > maxIterB)
            maxIterB = iter;
        avgIterB += iter;
    }
    qDebug() << "maxIter = " << maxIter << ", avg = " << avgIter/nTests << "maxIterSP = " << maxIterSP << ", avgSP = " << avgIterSP/nTests
                << "maxIterB = " << maxIterB << ", avgB = " << avgIterB/nTests;
}
}
#endif // TESTS_HPP
