#include "drawable.h"
#include "renderarea.h"
#include <assert.h>
Drawable::Drawable(RenderArea *renderArea) : QObject(NULL), renderArea(renderArea)
{
    assert(renderArea != NULL);
//    connect(renderArea, SIGNAL(painting()), this, SLOT(draw()));
}

Drawable::~Drawable()
{
//    disconnect(renderArea, SIGNAL(painting()), this, SLOT(draw()));
}



PointDrawable::~PointDrawable()
{

}

PointDrawable::PointDrawable(QPointF p, RenderArea *renderArea):Drawable(renderArea)
{
    pt.setX((p.x() + HALF_FIELD_MAXX) * renderArea->width() / (2.0 * HALF_FIELD_MAXX));
    pt.setY((-p.y() + HALF_FIELD_MAXY) * renderArea->height() / (2.0 * HALF_FIELD_MAXY));
}

void PointDrawable::draw()
{
//    QPainter painter(renderArea);
//    painter.save();
//    painter.setRenderHint(QPainter::Antialiasing);
//    QPen pen;
//    pen.setColor(Qt::black);
//    pen.setWidth(2);
//    painter.setPen(pen);
//    painter.drawEllipse(pt, 10, 10);
//    painter.restore();
}
