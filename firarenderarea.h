#ifndef FIRARENDERAREA_H
#define FIRARENDERAREA_H

#include <QWidget>
#include "pose.h"
#include "beliefstate.h"
#include <QMutex>
class FIRARenderArea : public QWidget
{
    Q_OBJECT
public:
    explicit FIRARenderArea(QWidget *parent = 0);
    void setEndPose(Pose p);
    Pose getEndPose();
    void setTrajectory(QPainterPath p);
    void toggleTrajectory(bool x) { drawTraj = x; this->update();}
    BeliefState *beliefStateSh;
    QMutex *bsMutex;
    Pose predictedPose; // predicted pose of some bot.
protected:
    void paintEvent(QPaintEvent *);
    void drawField(QPainter &painter);
    void drawBot(QPainter &painter, double botX, double botY, double botTheta, bool isHome);
    void drawBall(QPainter &painter, double ballX, double ballY);
    void drawPose(QPainter &painter); //index is 0 = end pose
    void drawTrajectory(QPainter &painter);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *event);
    bool scribbling;
    int x[1], y[1];    //
    double theta[1];   // All in QWidget coordinate system. Needs to be converted to our coordinates.
    QPainterPath traj;
    bool drawTraj;
signals:
    
public slots:
    
};

#endif // FIRARENDERAREA_H
