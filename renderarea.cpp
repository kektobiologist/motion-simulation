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
//    static const QPoint hourHand[3] =  {
//           QPoint(7, 8),
//           QPoint(-7, 8),
//           QPoint(0, -40)
//        };
//        static const QPoint minuteHand[3] =  {
//           QPoint(7, 8),
//           QPoint(-7, 8),
//           QPoint(0, -70)
//        };
//    QColor hourColor(127, 0, 127);
//    QColor minuteColor(0, 127, 127, 191);

//    int side = qMin(width(), height());
//    QTime time = QTime::currentTime();

    QPainter painter(this);
//    painter.setRenderHint(QPainter::Antialiasing);
//    painter.translate(width() / 2, height() / 2);
//    painter.scale(side / 200.0, side / 200.0);
//    QPen pen;
//    painter.save();
//    pen.setColor(hourColor);
//    painter.setPen(pen);
//    painter.rotate(30.0 * ((time.hour() + time.minute() / 60.0)));
//    painter.drawConvexPolygon(hourHand, 3);
//    painter.restore();

//    painter.save();
//    pen.setColor(minuteColor);
//    painter.setPen(pen);
//    painter.rotate(6.0 * (time.minute() + time.second() / 60.0));
//    painter.drawConvexPolygon(minuteHand, 3);
//    painter.restore();
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
    QRect bot; // is in the strategy coordinate system only.
    bot.setHeight(223); // 7.5 * fieldXConvert, what would be better? since fieldXConvert != fieldYConvert, its approx
    bot.setWidth(223);
    bot.moveCenter(QPoint(pose.x(), pose.y()));
    QLine midLine;
    midLine.setLine(bot.center().x(), bot.center().y(), bot.center().x(), bot.center().y()+bot.height()/2);
//    painter.translate(this->width()/2, this->height()/2);
    painter.scale(this->width()/(double)(2*HALF_FIELD_MAXX), this->height()/(double)(2*HALF_FIELD_MAXY));
    painter.rotate(pose.theta()*180/3.1415); // pathetic
    QPen pen;
    pen.setColor(Qt::black);
    pen.setWidth(4);
    painter.setPen(pen);
    painter.drawRect(bot);
    painter.drawLine(midLine);
    painter.restore();
}
