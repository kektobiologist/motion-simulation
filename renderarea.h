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
protected:
    void paintEvent(QPaintEvent *);
    void drawField(QPainter &painter);
    void drawBot(QPainter &painter);
signals:
    
public slots:
    
};

#endif // RENDERAREA_H
