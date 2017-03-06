#ifndef RENDERAREA_H
#define RENDERAREA_H

#include <QWidget>
#include "pose.h"
#include "beliefstate.h"
#include <QMutex>
#include <QPainterPath>

class RenderArea : public QWidget
{
    Q_OBJECT
    Pose pose; // pose of the robot to be drawn

public:
    explicit RenderArea(QWidget *parent = 0);
    void changePose(Pose p) {pose = p; update();}
    Pose getStartPose();
    Pose getEndPose();
    void setStartPose(Pose p);
    void setEndPose(Pose p);
    void setTrajectory(QPainterPath p);
    void toggleTrajectory(bool x) { drawTraj = x; this->update();}
protected:
    void paintEvent(QPaintEvent *);
    void drawField(QPainter &painter);
    void drawBot(QPainter &painter);
    void drawBot(QPainter &painter, double botX, double botY, double botTheta, bool isHome);
    void drawPose(QPainter &painter, int index); //index is 0 or 1, for start or end pose respectively.
    void drawTrajectory(QPainter &painter);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *event);
    bool scribbling;
    int x[2], y[2];    //
    double theta[2];   // All in QWidget coordinate system. Needs to be converted to our coordinates.
    int idx;
    QPainterPath traj;
    bool drawTraj;
signals:
    void painting();
public slots:
    void drawPoint(QPointF pt);
};

#endif // RENDERAREA_H
