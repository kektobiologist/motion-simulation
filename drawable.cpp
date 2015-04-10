#include "drawable.h"
#include "renderarea.h"
#include <assert.h>
Drawable::Drawable(RenderArea *renderArea) : QObject(NULL), renderArea(renderArea)
{

}

Drawable::~Drawable()
{

}



PointDrawable::~PointDrawable()
{
    disconnect(renderArea, SIGNAL(painting()), this, SLOT(draw()));
    disconnect(this, SIGNAL(drawPoint(QPointF)), renderArea, SLOT(drawPoint(QPointF)));
}

PointDrawable::PointDrawable(QPointF p, RenderArea *renderArea):Drawable(renderArea), pt(p)
{
    assert(renderArea != NULL);
    connect(renderArea, SIGNAL(painting()), this, SLOT(draw()));
    connect(this, SIGNAL(drawPoint(QPointF)), renderArea, SLOT(drawPoint(QPointF)));
}

void PointDrawable::draw()
{
    emit drawPoint(pt);
}
