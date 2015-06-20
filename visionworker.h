#ifndef VISIONTHREAD_H
#define VISIONTHREAD_H

#include <QObject>
#include <QThread>
#include "beliefstate.h"
#include <QMutex>
#include <sys/time.h>
#include <queue>
#include <utility>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include "opencv2/video/tracking.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

using namespace cv;
using namespace std;

using namespace std;
class VisionWorker : public QObject
{
    Q_OBJECT
public:
    explicit VisionWorker(QObject *parent = 0);
    void setup(QThread *cThread, BeliefState *bs, QMutex *bsMutex_, bool isTeamYellow = false);
signals:
    void newData();
public slots:
    void onEntry();
    void onExit();
private:
    ifstream myfile;
    ofstream writefile;
    //double deltaT = 0.016, omega_w =8, omega_u = 3.1623;
    KalmanFilter KF;
    Mat_<float> measurement;

    int detectionCount;
    static const int maxDetectionCount = 10; // after every these many detections, everything will be set as not detected
    QThread *myThread;
    BeliefState *bs;
    QMutex *bsMutex;
    bool isTeamYellow;
    // for velocity calc, just seeing last packet might give poor results
    // instead, look at old pose (and time) k packets ago
    std::queue<pair<BeliefState, double> > bsQ;
    // NOTE: can't be less than 1!
    static const int MAX_BS_Q = 2;
};

#endif // VISIONTHREAD_H
