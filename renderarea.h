#ifndef RENDERAREA_H
#define RENDERAREA_H

#include <QWidget>
#include "pose.h"
const int HALF_FIELD_MAXX              = 2975;
const int HALF_FIELD_MAXY              = 2050;
class RenderArea : public QWidget
{
    Q_OBJECT
    Pose pose; // pose of the robot to be drawn

public:
    explicit RenderArea(QWidget *parent = 0);
    void changePose(Pose p) {pose = p; update();}
    Pose getStartPose();
    Pose getEndPose();
protected:
    void paintEvent(QPaintEvent *);
    void drawField(QPainter &painter);
    void drawBot(QPainter &painter);
    void drawPose(QPainter &painter, int index); //index is 0 or 1, for start or end pose respectively.
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *event);
    bool scribbling;
    int x[2], y[2];    //
    double theta[2];   // All in QWidget coordinate system. Needs to be converted to our coordinates.
    int idx;
signals:
    
public slots:
    
};

#endif // RENDERAREA_H
