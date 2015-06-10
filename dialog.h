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
#include "controller-wrapper.hpp"
#include "visionworker.h"
#include "beliefstate.h"
#include <QMutex>
#include "serial.h"
#include "attacker.hpp"
#include "logging.pb.h"
#include <sys/time.h>
#include "simulation.hpp"
using namespace std;
namespace Ui {
class Dialog;
}




class Dialog : public QDialog
{
    Q_OBJECT
    
public:
    Vector2D<double> getVel;
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

    void on_writeLogButton_clicked();

    void on_trajCheckbox_toggled(bool checked);

    void on_trajButton_clicked();

    void on_traj2Button_clicked();

    void on_circleTrajButton_clicked();

    void on_splineChangeBtn_clicked();

    void on_trajSimButton_clicked();

    void on_interceptionButton_clicked();

    bool isFrontDirected(Pose botPos, Pose endPos);

public slots:
    void onCurIdxChanged(int idx); // idx is index of pose array, not botID (there's only 1 bot :/ )
    void onTimeout();
    void onAlgoTimeout();
    void onNewData();   // when new data for vision becomes available
private:
    QThread *visionThread;
    VisionWorker *vw;

    bool direction;
    BeliefState *beliefStateSh;
    QMutex *bsMutex;

    HAL::Serial comm;
    QTimer *algoTimer; //algoTimer for calling controller every 20ms. need to change to seperate thread.
    ControllerWrapper *algoController;
    // NOTE(arpit): not used in sim. Queue of predicted pose, size of q = PREDICTION_PACKET_DELAY. Needed because we need to display
    // old predictions side-by-side with the actual position of the bot.
    std::deque<Pose> predictedPoseQ;

    // don't need curIdx, simply read the position of the slider (otherwise there is duplicacy)
    Ui::Dialog *ui;
    QTimer *timer;
    // simulator
    Simulator sim;
    vector<FPair> functions;  // for display the spin box of choosing p2p controller
    // counter for counting num of packets sent:
    int counter;
    // so that comm.Write() commands don't overlap. ideally should also have a time gap between comm.Write() calls.
    QMutex *sendDataMutex;
    // structs for logging (actual bots, not sim)
    vector<Logging::SystemData> sysData;
    vector<Logging::ReceivedData> recvData;
    Logging::Log log;
    void readDataAndAppendToLog();
    Trajectory* traj;
    //SplineTrajectory* bi_traj; // for ball interception testing
    queue<Vector2D<double> > ballPoses;
    queue<Vector2D<double> > ballVels;
};

#endif // DIALOG_H
