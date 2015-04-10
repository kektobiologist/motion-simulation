#ifndef DRAWABLE_H
#define DRAWABLE_H

// a class for drawing objects on render area.
// currently implemented: point.
#include <QObject>
#include <QPainter>
#include "pose.h"
#include "beliefstate.h"
class RenderArea;
class Drawable : public QObject
{
    Q_OBJECT
protected:
    RenderArea *renderArea;
public:
    explicit Drawable(RenderArea *renderArea);
    virtual ~Drawable();
signals:

public slots:
    virtual void draw() = 0;
};

class PointDrawable : public Drawable {
    Q_OBJECT
private:
    QPointF pt;
public:
    virtual ~PointDrawable();
    PointDrawable(QPointF p, RenderArea *renderArea);
public slots:
    virtual void draw();
signals:
    void drawPoint(QPointF pt);
};

#endif // DRAWABLE_H
