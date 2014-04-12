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
    BeliefState *beliefStateSh;
    QMutex *bsMutex;
protected:
    void paintEvent(QPaintEvent *);
    void drawField(QPainter &painter);
    void drawBot(QPainter &painter, double botX, double botY, double botTheta, bool isHome);
    void drawBall(QPainter &painter, double ballX, double ballY);
    void drawPose(QPainter &painter); //index is 0 = end pose
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *event);
    bool scribbling;
    int x[1], y[1];    //
    double theta[1];   // All in QWidget coordinate system. Needs to be converted to our coordinates.
signals:
    
public slots:
    
};

#endif // FIRARENDERAREA_H
