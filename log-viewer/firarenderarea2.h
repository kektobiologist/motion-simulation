#ifndef FIRARENDERAREA_H
#define FIRARENDERAREA_H

#include <QWidget>
#include "pose.h"
#include "beliefstate.h"
#include <QMutex>
class FIRARenderArea2 : public QWidget
{
    Q_OBJECT
public:
    explicit FIRARenderArea2(QWidget *parent = 0);
    void setBeliefState(const BeliefState& bs_);
protected:
    void paintEvent(QPaintEvent *);
    void drawField(QPainter &painter);
    void drawBot(QPainter &painter, double botX, double botY, double botTheta, bool isHome);
    void drawBall(QPainter &painter, double ballX, double ballY);
    BeliefState bs;  // to draw bot etc. Needs to be updated externally.
signals:
    
public slots:
    
};

#endif // FIRARENDERAREA_H
