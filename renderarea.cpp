#include "renderarea.h"
#include <QPainter>
#include <QTime>
#include <QPen>
#include <QDebug>
RenderArea::RenderArea(QWidget *parent) :
    QWidget(parent)
{
}

void RenderArea::paintEvent(QPaintEvent *)
{
//    qDebug() << "Pose in render area: " << pose.x() << ", " << pose.y() << ", " << pose.theta()*180/3.141;
    QPainter painter(this);
    drawField(painter);
    drawBot(painter);
}

void RenderArea::drawField(QPainter &painter)
{
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    QRect field;
    field.setTopLeft(QPoint(0,0));
    field.setBottomRight(QPoint(this->width()-1, this->height()-1));
    painter.setPen(Qt::black);
    painter.setBrush(QBrush(Qt::green));
    painter.drawRect(field);
    painter.restore();
}

void RenderArea::drawBot(QPainter &painter)
{
    // size of field in qt is 595x410, same ratio as in ssl-vision.
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    QRectF bot; // is in the strategy coordinate system only.
    bot.setHeight(223); // 7.5 * fieldXConvert, what would be better? since fieldXConvert != fieldYConvert, its approx
    bot.setWidth(223);
    bot.moveCenter(QPoint(0, 0));
    QLine midLine;
    midLine.setLine(bot.center().x(), bot.center().y(), bot.center().x()+bot.width()/2, bot.center().y());
    painter.translate(this->width()/2, this->height()/2);
    painter.scale(this->width()/(double)(2*HALF_FIELD_MAXX), this->height()/(double)(2*HALF_FIELD_MAXY));
    painter.translate(pose.x(), pose.y());
    painter.rotate(pose.theta()*180/3.1415); // pathetic
    QPen pen;
    pen.setColor(Qt::black);
    pen.setWidth(4);
    painter.setPen(pen);
    painter.drawRect(bot);
    painter.drawLine(midLine);
    painter.restore();
}
