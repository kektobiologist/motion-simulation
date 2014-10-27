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
protected:
    void paintEvent(QPaintEvent *);
    void drawField(QPainter &painter);
    void drawBot(QPainter &painter, double botX, double botY, double botTheta, bool isHome);
    void drawBall(QPainter &painter, double ballX, double ballY);
signals:
    
public slots:
    
};

#endif // FIRARENDERAREA_H
