#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "pose.h"
#include <QTimer>
namespace Ui {
class Dialog;
}

// Constants required by generateControl, directly copied from most recent version of kgpkubs.
const float MAX_BOT_LINEAR_VEL_CHANGE  = 3;
const float MAX_BOT_SPEED              = 80.0;
const int BOT_POINT_THRESH             = 147;
const int CLEARANCE_PATH_PLANNER       = 300;

const double timeLC = 16e-3;
const int NUMTICKS = 1000;

#define SGN(x) (((x)>0)?1:(((x)<0)?(-1):0))
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

public slots:
    void onCurIdxChanged(int idx); // idx is index of pose array, not botID (there's only 1 bot :/ )
    void onTimeout();
private:
    int curIdx;
    Ui::Dialog *ui;
    QTimer *timer;
    Pose poses[NUMTICKS];
    void generateControl(Pose initialPose, Pose finalPose, int &vl, int &vr, int clearance = CLEARANCE_PATH_PLANNER);
    void simulate(Pose startPose, Pose endPose);


    void drawControlArc(int idx, Pose endPose);
};

#endif // DIALOG_H
