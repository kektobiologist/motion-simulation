#include "visionworker.h"
#include "robocup_ssl_client.h"
#include "timer.h"
#include <QDebug>
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "pose.h"
#include "vision-velocity.hpp"
#include <assert.h>

#include <iostream>
#include <stdio.h>
#include "opencv2/video/tracking.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

using namespace cv;
using namespace std;

void printRobotInfo(const SSL_DetectionRobot & robot) {
    printf("CONF=%4.2f ", robot.confidence());
    if (robot.has_robot_id()) {
        printf("ID=%3d ",robot.robot_id());
    } else {
        printf("ID=N/A ");
    }
    printf(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ",robot.height(),robot.x(),robot.y());
    if (robot.has_orientation()) {
        printf("ANGLE=%6.3f ",robot.orientation());
    } else {
        printf("ANGLE=N/A    ");
    }
    printf("RAW=<%8.2f,%8.2f>\n",robot.pixel_x(),robot.pixel_y());
}
inline void linearTransform(double &x, double &y, double &newangle)
{
  double tempx, tempy;
  tempx = (y*(-HALF_FIELD_MAXX))/(2050.0);
  tempy = ((x + 3025.0/2)*(HALF_FIELD_MAXY))/(3025.0/2.0);
  x = tempx;
  y = tempy;
  newangle = normalizeAngle(newangle+PI/2);
  //y=tempy-800;
}
VisionWorker::VisionWorker(QObject *parent) :
    QObject(parent)
{
    detectionCount = 0;
    isTeamYellow = false;
       omega_w =8;
       omega_u = 3.1623;
       KFx =  KalmanFilter(3, 2, 0);
       measurementx =  Mat_<double>(2,1);
       KFy =  KalmanFilter(3, 2, 0);
       measurementy =  Mat_<double>(2,1);
       measurementx.setTo(Scalar(0));
       KFx.statePost.at<double>(0) = 0.0; // X
       KFx.statePost.at<double>(1) = 0.0; // dX
       KFx.statePost.at<double>(2) = 0.0; // theta
       KFx.transitionMatrix = (Mat_<double>(3, 3) << 1,1,0,   0,1,0,  0,0,1  ); //F
       KFx.measurementMatrix = (Mat_<double>(2, 3) << 1,0,0, 0,0,1  );  //H
       KFx.processNoiseCov = (Mat_<double>(3, 3) << 1,0,0, 0,0.1,0, 0,0,0.1);
       KFx.processNoiseCov *=pow(omega_w,2);
       setIdentity(KFx.measurementNoiseCov, Scalar::all(pow(omega_u,2)));
       setIdentity(KFx.errorCovPost, Scalar::all(50));

       KFy.statePost.at<float>(0) = 0; // X
       KFy.statePost.at<float>(1) = 0; // dX
       KFy.statePost.at<float>(2) = 0; // theta
       KFy.transitionMatrix = (Mat_<float>(3, 3) << 1,1,0,   0,1,0,  0,0,1  ); //F
       KFy.measurementMatrix = (Mat_<float>(2, 3) << 1,0,0, 0,0,1  );  //H
       KFy.processNoiseCov = (Mat_<float>(3, 3) << 1,0,0, 0,0.1,0, 0,0,0.1);
       KFy.processNoiseCov *=pow(omega_w,2);
       setIdentity(KFy.measurementNoiseCov, Scalar::all(pow(omega_u,2)));
       setIdentity(KFy.errorCovPost, Scalar::all(50));

}

void VisionWorker::setup(QThread *cThread, BeliefState *bs_, QMutex *bsMutex_, bool isTeamYellow_)
{
    myThread = cThread;
    bs = bs_;
    bsMutex = bsMutex_;
    isTeamYellow = isTeamYellow_;
    connect(cThread, SIGNAL(started()), this, SLOT(onEntry()));
    connect(cThread, SIGNAL(destroyed()), this, SLOT(onExit()));
    for (int i = 0; i < MAX_BS_Q; i++) {
        bsQ.push(make_pair(BeliefState(), 0));
    }
}

void VisionWorker::onEntry()
{
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;

    qDebug() << "here" << endl;
    while(true) {
        if (client.receive(packet)) {
            if (packet.has_detection()) {
                double nowTime = packet.detection().t_capture();
                double timeMs = (nowTime - bsQ.front().second)*1000.0;
                if (timeMs <= 0) {
                    timeMs = 0.001; // this can happen at the beginning. who cares.
                    qDebug() << "timeMs is negative!";
                }
                detectionCount++;
                if(detectionCount == maxDetectionCount) {
                    detectionCount = 0;
                    bsMutex->lock();
                    bs->ballIsPresent = false;
                    for(int i = 0; i < 5; i++) {
                        bs->homeIsPresent[i] = false;
                        bs->awayIsPresent[i] = false;
                    }
                    bsMutex->unlock();
                }
                SSL_DetectionFrame detection = packet.detection();
                //Display the contents of the robot detection results:
//                double t_now = GetTimeSec();

                int balls_n = detection.balls_size();
                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();
                float rhoKFx, DrhoKFx, thetaKFx, xCoordinateOfObject,newx,lastPosx = 0;
                //Ball info:
                if(balls_n) {
                    // only take 1st ball into account;
                    SSL_DetectionBall ball = detection.balls(0);
                    double ballX, ballY, dummy = 0;
                    float ballVx, ballVy;
                    ballX = ball.x();
                    ballY = ball.y();
                    linearTransform(ballX, ballY, dummy);
                    VisionVelocity::calcBallVelocity(ballX - bsQ.front().first.ballX, ballY - bsQ.front().first.ballY, timeMs, ballVx, ballVy);
                    bsMutex->lock();
                    bs->ballX = ballX;
                    bs->ballY = ballY;
                    bs->ballIsPresent = true;
                    bs->ballVx = ballVx;
                    bs->ballVy = ballVy;
                    bsMutex->unlock();
                    float avgvx=0,avgvy=0;

                        velxq.push_back(ballVx);
                        velyq.push_back(ballVy);
                        for(int i=1;i<4;i++)
                        {
                            if(i>velxq.size()) break;
                            avgvx+=velxq[i-1];
                            avgvy+=velyq[i-1];

                        }
                        bs->ballVx = avgvx/velxq.size();
                        bs->ballVy = avgvy/velyq.size();
                        while(velxq.size() > 3)
                            velxq.pop_front();
                        while(velyq.size() > 3)
                            velyq.pop_front();

                    ////////////////////////////////////////////
//                              rhoKFx = KFx.statePost.at<double>(0);  // rho
//                               DrhoKFx = KFx.statePost.at<double>(1); // d rho
//                               thetaKFx  = 0.;//KF.statePost.at<float>(2); // theta
//                                double Dcosx  = tan(thetaKFx)*(1/cos(thetaKFx));
//                                // Jacobina of transfer function => F
//                              // changing here
//                               KFx.transitionMatrix = (Mat_<double>(3, 3) << 1.,(timeMs*0.001)/cos(thetaKFx),DrhoKFx*timeMs*Dcosx,   0.,1.,0., 0.,0.,1.); //(DrhoKF*deltaT*sin(thetaKF))/pow(cos(thetaKF),2)
//                            //   Mat predictionx = KF.predict();
//                               KFx.statePre.at<float>(0) = rhoKFx + DrhoKFx * timeMs / cos(thetaKFx);
//                               KFx.statePre.at<float>(1) = DrhoKFx;
//                               KFx.statePre.at<float>(2) = thetaKFx;

//                               // Update
//                              // myfile >> xCoordinateOfObject >> y >> newx >> newy >> velx >> vely;
//                               measurement.at<float>(0) = ballX;
//                               measurement.at<float>(1) = 0;
//                              // lastRho = rightLane[0];
//                               Mat estimatedx = KFx.correct(measurement);
//                               KFx.temp5.at<float>(0) = measurement.at<float>(0) - KFx.statePre.at<float>(0);
//                               KFx.temp5.at<float>(1) = measurement.at<float>(1) - KFx.statePre.at<float>(2);
//                               KFx.statePost = KFx.statePre + KFx.gain * KFx.temp5;
                    /////////////////////////////////////////////////
//                               float rhoKFy, DrhoKFy, thetaKFy, yCoordinateOfObject,newy,lastPosy = 0;

//                                          rhoKFy = KFy.statePost.at<float>(0);  // rho
//                                          DrhoKFy = KFy.statePost.at<float>(1); // d rho
//                                          thetaKFy  = 0;//KF.statePost.at<float>(2); // theta     double Dcos  = tan(thetaKF)*(1/cos(thetaKF));
//                                           double Dcosy  = tan(thetaKFy)*(1/cos(thetaKFy));
//                                           // Jacobina of transfer function => F

//                                          // changing here
//                                          KFy.transitionMatrix = (Mat_<float>(3, 3) << 1,timeMs/cos(thetaKFy),DrhoKFy*timeMs*Dcosy,   0,1,0, 0,0,1); //(DrhoKF*deltaT*sin(thetaKF))/pow(cos(thetaKF),2)
//                                          Mat predictiony = KFy.predict();
//                                          KFy.statePre.at<float>(0) = rhoKFy + DrhoKFy * timeMs / cos(thetaKFy);
//                                          KFy.statePre.at<float>(1) = DrhoKFy;
//                                          KFy.statePre.at<float>(2) = thetaKFy;

//                                          // Update
//                                         // myfile >> xCoordinateOfObject >> y >> newx >> newy >> velx >> vely;
//                                          measurement.at<float>(0) = yCoordinateOfObject;
//                                          measurement.at<float>(1) = 0;
//                                         // lastRho = rightLane[0];
//                                          Mat estimatedy = KFy.correct(measurement);
//                                          KFy.temp5.at<float>(0) = measurement.at<float>(0) - KFy.statePre.at<float>(0);
//                                          KFy.temp5.at<float>(1) = measurement.at<float>(1) - KFy.statePre.at<float>(2);
//                                          KFy.statePost = KFy.statePre + KFy.gain * KFy.temp5;

//                                          lastPosy = yCoordinateOfObject;

                    ////////////////////////////////////////////////

                }
                //Blue robot info:
                // currently only handling blue robot info as home
                for (int i = 0; i < robots_blue_n; i++) {
                    double botX, botY, botTheta;
                    float botVl, botVr;
                    SSL_DetectionRobot robot = detection.robots_blue(i);
                    int botId = robot.robot_id();
                    botX = robot.x();
                    botY = robot.y();
                    if(robot.has_orientation())
                        botTheta = robot.orientation();
                    else
                        botTheta = 0;
                    linearTransform(botX, botY, botTheta);
                    if (isTeamYellow) {
                        VisionVelocity::BotPose p1(bsQ.front().first.awayX[botId], bsQ.front().first.awayY[botId], bsQ.front().first.awayTheta[botId]);
                        VisionVelocity::BotPose p2(botX, botY, botTheta);
                        VisionVelocity::calcBotVelocity(p1, p2, timeMs, botVl, botVr);
                        bsMutex->lock();
                        bs->awayX[botId] = botX;
                        bs->awayY[botId] = botY;
                        bs->awayTheta[botId] = botTheta;
                        bs->awayVl[botId] = botVl;
                        bs->awayVr[botId] = botVr;
                        bs->awayIsPresent[botId] = true;
                        bsMutex->unlock();
                    } else {
                        VisionVelocity::BotPose p1(bsQ.front().first.homeX[botId], bsQ.front().first.homeY[botId], bsQ.front().first.homeTheta[botId]);
                        VisionVelocity::BotPose p2(botX, botY, botTheta);
                        VisionVelocity::calcBotVelocity(p1, p2, timeMs, botVl, botVr);
                        bsMutex->lock();
                        bs->homeX[botId] = botX;
                        bs->homeY[botId] = botY;
                        bs->homeTheta[botId] = botTheta;
                        bs->homeVl[botId] = botVl;
                        bs->homeVr[botId] = botVr;
                        bs->homeIsPresent[botId] = true;
                        bsMutex->unlock();
                    }
                }

                //Yellow robot info:
                // currently yellow is away
                for (int i = 0; i < robots_yellow_n; i++) {
                    double botX, botY, botTheta;
                    float botVl, botVr;
                    SSL_DetectionRobot robot = detection.robots_yellow(i);
                    int botId = robot.robot_id();
                    botX = robot.x();
                    botY = robot.y();
                    if(robot.has_orientation())
                        botTheta = robot.orientation();
                    else
                        botTheta = 0;
                    linearTransform(botX, botY, botTheta);
                    if (isTeamYellow) {
                        VisionVelocity::BotPose p1(bsQ.front().first.homeX[botId], bsQ.front().first.homeY[botId], bsQ.front().first.homeTheta[botId]);
                        VisionVelocity::BotPose p2(botX, botY, botTheta);
                        VisionVelocity::calcBotVelocity(p1, p2, timeMs, botVl, botVr);
                        bsMutex->lock();
                        bs->homeVx[botId]= (botX - bs->homeX[botId])/(timeMs*0.001);
                        bs->homeVy[botId]= (botY - bs->homeY[botId])/(timeMs*0.001);
                        bs->homeX[botId] = botX;
                        bs->homeY[botId] = botY;
                        bs->homeTheta[botId] = botTheta;
                        bs->homeVl[botId] = botVl;
                        bs->homeVr[botId] = botVr;
                        bs->homeIsPresent[botId] = true;
                        bsMutex->unlock();
                    } else {
                        VisionVelocity::BotPose p1(bsQ.front().first.awayX[botId], bsQ.front().first.awayY[botId], bsQ.front().first.awayTheta[botId]);
                        VisionVelocity::BotPose p2(botX, botY, botTheta);
                        VisionVelocity::calcBotVelocity(p1, p2, timeMs, botVl, botVr);
                        bsMutex->lock();
                        bs->awayX[botId] = botX;
                        bs->awayY[botId] = botY;
                        bs->awayTheta[botId] = botTheta;
                        bs->awayVl[botId] = botVl;
                        bs->awayVr[botId] = botVr;
                        bs->awayIsPresent[botId] = true;
                        bsMutex->unlock();
                    }
                }                
                bsQ.pop();
                bsMutex->lock();
                bsQ.push(make_pair(*bs, nowTime));
                bsMutex->unlock();
                emit newData();
            }

        }
    }
}


void VisionWorker::onExit()
{
}
