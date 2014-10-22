#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "pose.h"
#include <QTimer>
#include <vector>
#include <map>
#include <algorithm>
#include <utility>
#include <deque>
#include <queue>
#include "controllers.h"
#include "visionworker.h"
#include "beliefstate.h"
#include <QMutex>
#include "serial.h"
using namespace std;
namespace Ui {
class Dialog;
}


struct RegData {
    double rho, gamma, delta;
    double timeMs;
    RegData():rho(0), gamma(0), delta(0), timeMs(0){}
    RegData(double rho, double gamma, double delta, double timeMs):rho(rho), gamma(gamma), delta(delta), timeMs(timeMs){}
};

class Dialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();
    
private slots:
    void on_startButton_clicked();

    void on_pauseButton_clicked();

    void on_resetButton_clicked();

    void on_horizontalSlider_sliderMoved(int position);


    void on_simButton_clicked();

    void on_batchButton_clicked();

    void on_startSending_clicked();

    void on_stopSending_clicked();

    void on_receiveButton_clicked();

    void on_clearButton_clicked();

public slots:
    void onCurIdxChanged(int idx); // idx is index of pose array, not botID (there's only 1 bot :/ )
    void onTimeout();
    void onAlgoTimeout();
private:
    QThread *visionThread;
    VisionWorker *vw;

    BeliefState *beliefStateSh;
    QMutex *bsMutex;

    HAL::Serial comm;
    QTimer *algoTimer; //algoTimer for calling controller every 20ms. need to change to seperate thread.
    ControllerWrapper *algoController;
    // NOTE(arpit): not used in sim. Queue of predicted pose, size of q = PREDICTION_PACKET_DELAY. Needed because we need to display
    // old predictions side-by-side with the actual position of the bot.
    std::queue<Pose> predictedPoseQ;

    // don't need curIdx, simply read the position of the slider (otherwise there is duplicacy)
    Ui::Dialog *ui;
    QTimer *timer;
    Pose poses[NUMTICKS];
    int vls[NUMTICKS], vrs[NUMTICKS];
    double simulate(Pose startPose, Pose endPose, FType func, bool isBatch = false); // implements delay control logic, for any given controller. (I removed the old simulate function that did not use wrapper)
                                                                                     // returns the time(ms) to reach endPose. A dist threshold is taken, no angle considerations yet.
    void batchSimulation(FType fun);
    vector<FPair> functions;
    void drawControlArc(int idx, Pose endPose);
    void regression(vector<RegData> func);

    // functions for GA
    double fitnessFunction(double k1, double k2, double k3); // runs the PolarBasedGA function with k1, k2, k3 values.

    // counter for counting num of packets sent:
    int counter;
};

#endif // DIALOG_H
