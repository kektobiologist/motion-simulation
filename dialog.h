#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "pose.h"
#include <QTimer>
#include <vector>
#include <map>
#include <algorithm>
#include <utility>

using namespace std;
namespace Ui {
class Dialog;
}

// Constants required by generateControl, directly copied from most recent version of kgpkubs.
const float MAX_BOT_LINEAR_VEL_CHANGE  = 3;
const float MAX_BOT_SPEED              = 80.0;
const int BOT_POINT_THRESH             = 147;
const int CLEARANCE_PATH_PLANNER       = 400;

const double timeLC = 16e-3;
const int NUMTICKS = 1000;
#define SGN(x) (((x)>0)?1:(((x)<0)?(-1):0))
class Dialog : public QDialog
{
    typedef void(Dialog::*FType)(Pose, Pose, int &, int &);
    typedef std::pair<QString, FType> FPair;
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

public slots:
    void onCurIdxChanged(int idx); // idx is index of pose array, not botID (there's only 1 bot :/ )
    void onTimeout();
private:
    // don't need curIdx, simply read the position of the slider (otherwise there is duplicacy)
    Ui::Dialog *ui;
    QTimer *timer;
    Pose poses[NUMTICKS];
    int vls[NUMTICKS], vrs[NUMTICKS];
    void kgpkubs(Pose initialPose, Pose finalPose, int &vl, int &vr);
    void CMU(Pose s, Pose e, int &vl, int &vr);
    void PController(Pose s, Pose e, int &vl, int &vr);
    void PolarBased(Pose s, Pose e, int &vl, int &vr);
    void simulate(Pose startPose, Pose endPose, FType fun);
    vector<FPair> functions;
    void drawControlArc(int idx, Pose endPose);
};

#endif // DIALOG_H
