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
#include "controllers.h"
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

public slots:
    void onCurIdxChanged(int idx); // idx is index of pose array, not botID (there's only 1 bot :/ )
    void onTimeout();
private:
    class DelayController {
        FType fun;
        deque<pair<int,int> > uq; // controls queue.
        int k;                    // the num of packet delay
    public:
        DelayController(FType fun, int k):fun(fun), k(k) {
            for(int i = 0; i < k; i++)
                uq.push_back(make_pair<int,int>(0,0));
        }
        void genControls(Pose s, Pose e, int &vl, int &vr, double prevSpeed) {
            Pose x = s;
            for(deque<pair<int,int> >::iterator it = uq.begin(); it != uq.end(); it++) {
                x.updateNoDelay(it->first, it->second, timeLC);
            }
            (*fun)(x, e, vl, vr, prevSpeed);
            if(uq.size()) {
                uq.pop_front();
                uq.push_back(make_pair<int,int>(vl, vr));
            }
        }
    };

    // don't need curIdx, simply read the position of the slider (otherwise there is duplicacy)
    Ui::Dialog *ui;
    QTimer *timer;
    Pose poses[NUMTICKS];
    int vls[NUMTICKS], vrs[NUMTICKS];
    double simulate(Pose startPose, Pose endPose, FType func); // returns the time(ms) to reach endPose. A dist threshold is taken, no angle considerations yet.
    double simulateDelayController(Pose startPose, Pose endPose, FType func, bool isBatch = false); // implements delay control logic, for any given controller.
    void batchSimulation(FType fun);
    vector<FPair> functions;
    void drawControlArc(int idx, Pose endPose);
    void regression(vector<RegData> func);

    // functions for GA
    double fitnessFunction(double k1, double k2, double k3); // runs the PolarBasedGA function with k1, k2, k3 values.
};

#endif // DIALOG_H
