#ifndef BALLINTERCEPTION_HPP
#define BALLINTERCEPTION_HPP

#include <math.h>
#include "pose.h"
#include "velocity-profile.hpp"
#include "trajectory.hpp"
#include "splines.hpp"
#include "controlpoint-optimization.hpp"
#include "trajectory-generators.hpp"

namespace BallInterception {

Vector2D<double> predictBallPose(Vector2D<double> ballPos, Vector2D<double> ballVel, double mu, double timeOfPrediction){
    Vector2D<double> bPos;
    double a = 100 / mu;
    bPos.x = ballPos.x + timeOfPrediction*ballVel.x - 0.5 * a * timeOfPrediction * timeOfPrediction;
    bPos.y = ballPos.y + timeOfPrediction*ballVel.y - 0.5 * a * timeOfPrediction * timeOfPrediction;

    while (bPos.x > HALF_FIELD_MAXX || bPos.x < -HALF_FIELD_MAXX || bPos.y > HALF_FIELD_MAXY || bPos.y < -HALF_FIELD_MAXY) {
        if (bPos.y > HALF_FIELD_MAXY) {
            bPos.y = 2 * HALF_FIELD_MAXY - bPos.y;
        }
        if (bPos.y < -HALF_FIELD_MAXY) {
            bPos.y = - (2 * HALF_FIELD_MAXY + bPos.y);
        }
        if (bPos.x > HALF_FIELD_MAXX) {
            bPos.x = 2 * HALF_FIELD_MAXX - bPos.x;
        }
        if (bPos.x < -HALF_FIELD_MAXX) {
            bPos.x = - (2 * HALF_FIELD_MAXX + bPos.x);
        }
    }

    return bPos;
}

double getBotBallDist(Pose botPos, Vector2D<double> ballPos) {
    return sqrt((botPos.x() - ballPos.x)*(botPos.x() - ballPos.x) + (botPos.y() - ballPos.y)*(botPos.y() - ballPos.y));
}

SplineTrajectory* getIntTraj(Pose botPosStart, Vector2D<double> ballPos, Vector2D<double> ballVel, Vector2D<double> botVel, double mu) {
    Vector2D<double> predictedBallPos;
    double error = 0.1;
    double T2 = 6.0;
    double T1 = 0.0;
    double S = 1.0;
    Vector2D<double> goalCentre(HALF_FIELD_MAXX, 0);
    SplineTrajectory *st = NULL;
    double T = T1;
    while (1) {
        if (st)
            delete st;
        predictedBallPos = predictBallPose(ballPos, ballVel, mu, T);
        double endTheta = atan2(goalCentre.y - predictedBallPos.y, goalCentre.x - predictedBallPos.x);
        Pose endPose(predictedBallPos.x, predictedBallPos.y, endTheta);
        // add a cp behind the ball pos, distance of 500
      //  Pose cp1(predictedBallPos.x+500*cos(endTheta+M_PI), predictedBallPos.y+500*sin(endTheta+M_PI), 0);
        vector<Pose> midPoints;
       // midPoints.push_back(cp1);
       st = TrajectoryGenerators::cubic(botPosStart, endPose, 0,0, 40,40 , midPoints);
       //st = TrajectoryGenerators::cubic(botPosStart, endPose, botVel.x, botVel.y, 30, 30, midPoints);
        double t = st->totalTime();
        if (t < T)
            break;
        T += S;
        if (T >= 6.0)
            break;
    }
    if (T <= 6.0) {
        T2 = T;
        T1 = T - S;
    }
    while (1) {
        // predictedBallPos: strategy coordinates
        double mid = (T1+T2)/2;
        predictedBallPos = predictBallPose(ballPos, ballVel, mu, mid);
        double endTheta = atan2(goalCentre.y - predictedBallPos.y, goalCentre.x - predictedBallPos.x);
        Pose endPose(predictedBallPos.x, predictedBallPos.y, endTheta);
        if (st)
            delete st;
        // add a cp behind the ball pos, distance of 500
      //  Pose cp1(predictedBallPos.x+500*cos(endTheta+M_PI), predictedBallPos.y+500*sin(endTheta+M_PI), 0);
        vector<Pose> midPoints;
       // midPoints.push_back(cp1);
       st = TrajectoryGenerators::cubic(botPosStart, endPose, 0,0, 40, 40, midPoints);
        //st = TrajectoryGenerators::cubic(botPosStart, endPose, botVel.x, botVel.y, 30, 30, midPoints);
        double t = st->totalTime();
        qDebug() << "mid = " << mid << ", bot-ka-time = " << t;
        if (fabs(t-mid) < error)
            break;
        if (t > mid) {
            T1 = mid;
        } else if (t < mid) {
            T2 = mid;
        }
        if (fabs(T2-T1) < error) {
            qDebug() << "T2, T1 almost same = " << T1 <<", t = " << t;
            break;
        }
    }
    return st;
}
}

#endif // BALLINTERCEPTION_HPP
