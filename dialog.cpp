#include "dialog.h"
#include "ui_dialog.h"
#include "geometry.h"
#include <stdio.h>
#include <assert.h>
#include <QTimer>
#include <QDebug>
Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    timer = new QTimer();
    //img = cvCreateImage(cvSize(ui->drawArea->width(), ui->drawArea->height()), 8, 3);
    simulate(Pose(HALF_FIELD_MAXX/2,HALF_FIELD_MAXY/2,PI/8), Pose(0,0,0));
    ui->horizontalSlider->setRange(0, NUMTICKS-1);
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(onCurIdxChanged(int)));
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    onCurIdxChanged(0);
}

Dialog::~Dialog()
{
    delete ui;
}


void Dialog::generateControl(Pose initialPose, Pose finalPose, int &vl, int &vr, int clearance)
{
    static float prevVel = 0;
    Vector2D<int> initial(initialPose.x(), initialPose.y());
    Vector2D<int> final(finalPose.x(), finalPose.y());
    double curSlope = initialPose.theta();
    double finalSlope = finalPose.theta();
    double theta = normalizeAngle(Vector2D<int>::angle(final, initial));
    double phiStar = finalSlope;
    double phi = curSlope;
    //directionalAngle = (cos(atan2(final.y - initial.y, final.x - initial.x) - curSlope) >= 0) ? curSlope : normalizeAngle(curSlope - PI);
    int dist = Vector2D<int>::dist(final, initial);  // Distance of next waypoint from the bot
    double alpha = normalizeAngle(theta - phiStar);
    double beta = (fabs(alpha) < fabs(atan2(clearance, dist))) ? (alpha) : SGN(alpha) * atan2(clearance, dist);
    double thetaD = normalizeAngle(theta + beta);
    float delta = normalizeAngle(thetaD - phi);
    double r = (sin(delta) * sin(delta)) * SGN(tan(delta));
    double t = (cos(delta) * cos(delta)) * SGN(cos(delta));
    if(!(t >= -1.0 && t <= 1.0 && r >= -1.0 && r <= 1.0)) {
     printf("what\n");
     assert(0);
    }

    float fTheta = asin(sqrt(fabs(r)));
    fTheta = 1 - fTheta/(PI/2);
    fTheta = pow(fTheta,2.2) ;
    float fDistance = (dist > BOT_POINT_THRESH*3) ? 1 : dist / ((float) BOT_POINT_THRESH *3);
    float fTot = fDistance * fTheta;
    fTot = 0.2 + fTot*(1-0.2);
    float profileFactor = MAX_BOT_SPEED * fTot;
    if(fabs(r)<0.11)
      profileFactor*=2;
    {
      if(fabs(profileFactor-prevVel)>MAX_BOT_LINEAR_VEL_CHANGE)
      {
        if(profileFactor>prevVel)
            profileFactor=prevVel+MAX_BOT_LINEAR_VEL_CHANGE;
        else
            profileFactor=prevVel-MAX_BOT_LINEAR_VEL_CHANGE;
      }
      prevVel=profileFactor;
    }
    if(profileFactor>1.5*MAX_BOT_SPEED)
      profileFactor = 1.5*MAX_BOT_SPEED;
    else if(profileFactor <-1.5*MAX_BOT_SPEED)
      profileFactor = -1.5*MAX_BOT_SPEED;
    prevVel=profileFactor;
    r *= 0.5*profileFactor;
    t *= profileFactor;

    vl = t-r;
    vr = t+r;
}

void Dialog::simulate(Pose startPose, Pose endPose)
{
    poses[0] = startPose;
    //simulating behaviour for all ticks at once
    for(int i=1; i < NUMTICKS; i++)
    {
        poses[i] = poses[i-1];
        int vl, vr;
        generateControl(poses[i], endPose, vl, vr);
        qDebug() << "vl, vr = " << vl << ", " << vr;
        poses[i].update(vl, vr, timeLC);
    }
}


void Dialog::on_startButton_clicked()
{
    timer->start(16);
}

void Dialog::on_pauseButton_clicked()
{
    timer->stop();
}

void Dialog::on_resetButton_clicked()
{
    timer->stop();
    ui->horizontalSlider->setValue(0);
}

void Dialog::on_horizontalSlider_sliderMoved(int position)
{
    timer->stop();
//    onCurIdxChanged(position);
}

void Dialog::onCurIdxChanged(int idx)
{
    ui->renderArea->changePose(poses[idx]);
    qDebug() << "Pose: " << poses[idx].x() << ", " << poses[idx].y() << ", " << poses[idx].theta()*180/PI;
}

void Dialog::onTimeout()
{
    int idx = ui->horizontalSlider->value();
    idx++;
    if(idx >= NUMTICKS) {
        timer->stop();
        return;
    }
    if(idx < 0 || idx >= NUMTICKS) {
        qDebug() << "Error! idx = " << idx << " and is out of range!";
        return;
    }    
    ui->horizontalSlider->setValue(idx);
//    onCurIdxChanged(curIdx+1);
}

