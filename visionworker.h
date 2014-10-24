#ifndef VISIONTHREAD_H
#define VISIONTHREAD_H

#include <QObject>
#include <QThread>
#include "beliefstate.h"
#include <QMutex>
#include <sys/time.h>

class VisionWorker : public QObject
{
    Q_OBJECT
public:
    explicit VisionWorker(QObject *parent = 0);
    void setup(QThread *cThread, BeliefState *bs, QMutex *bsMutex_);
signals:
    void newData();
public slots:
    void onEntry();
    void onExit();
private:
    int detectionCount; //
    static const int maxDetectionCount = 10; // after every these many detections, everything will be set as not detected
    QThread *myThread;
    BeliefState *bs;
    QMutex *bsMutex;
    struct timeval lastTime;
};

#endif // VISIONTHREAD_H
