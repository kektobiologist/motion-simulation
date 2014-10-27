#include "firarenderarea.h"
#include <QPainter>
#include <QTime>
#include <QPen>
#include <QDebug>
#include <QEvent>
#include <QMouseEvent>
#include <math.h>
#include <assert.h>
FIRARenderArea::FIRARenderArea(QWidget *parent) :
    QWidget(parent)
{
    scribbling = false;
    x[0] = 605*2/3;
    y[0] = 410*2/3;
    theta[0] = 3.1418/8;
}

void FIRARenderArea::paintEvent(QPaintEvent *)
{
//    qDebug() << "Pose in render area: " << pose.x() << ", " << pose.y() << ", " << pose.theta()*180/3.141;
    QPainter painter(this);
    drawField(painter);
    BeliefState bs;
    if(bsMutex) {
        bsMutex->lock();
        if(beliefStateSh)
            bs = *beliefStateSh;
        bsMutex->unlock();
    }
    for(int i = 0; i < 5; i++) {
        if(bs.homeIsPresent[i]) {
            drawBot(painter, bs.homeX[i], bs.homeY[i], bs.homeTheta[i], true);
        }
        if(bs.awayIsPresent[i]) {
            drawBot(painter, bs.awayX[i], bs.awayY[i], bs.awayTheta[i], false);
        }
    }
    drawBot(painter, predictedPose.queryX(), predictedPose.queryY(), predictedPose.queryTheta(), false);
    if(bs.ballIsPresent) {
        drawBall(painter, bs.ballX, bs.ballY);
    }
    drawPose(painter);
}

void FIRARenderArea::drawField(QPainter &painter)
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

void FIRARenderArea::drawBot(QPainter &painter, double botX, double botY, double botTheta, bool isHome)
{
    // size of field in qt is 605x410, same ratio as in ssl-vision.
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
    painter.translate(botX, -botY);
    painter.rotate(-botTheta*180/3.1415); // pathetic
    QPen pen;
    if(isHome)
        pen.setColor(Qt::blue);
    else
        pen.setColor(Qt::red);
    pen.setWidth(10);
    painter.setPen(pen);
    painter.drawRect(bot);
    painter.drawLine(midLine);
    painter.restore();
}

void FIRARenderArea::drawBall(QPainter &painter, double ballX, double ballY)
{
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    QPen pen;
    pen.setColor("orange");
    pen.setWidth(5);
    QBrush brush;
    brush.setColor("orange");
    brush.setStyle(Qt::SolidPattern);
    painter.setBrush(brush);
    painter.setPen(pen);
    painter.translate(this->width()/2, this->height()/2);
    painter.scale(this->width()/(double)(2*HALF_FIELD_MAXX), this->height()/(double)(2*HALF_FIELD_MAXY));
    painter.translate(ballX, -ballY);
    // dia = 4.2 cm
    painter.drawEllipse(QPointF(0, 0), 4.2/2*Pose::fieldXConvert, 4.2/2*Pose::fieldYConvert);
    painter.restore();
}

void FIRARenderArea::drawPose(QPainter &painter)
{
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    QPen pen;
    pen.setColor(Qt::black);
    pen.setWidth(2);
    painter.setPen(pen);
    int x2, y2;
    x2 = x[0] + cos(theta[0])*50;
    y2 = y[0] + sin(theta[0])*50;
    painter.drawLine(x[0], y[0], x2, y2);
    pen.setColor(Qt::red);
    painter.setPen(pen);
    painter.drawEllipse(QPointF(x[0], y[0]), 10, 10);
    painter.restore();
}
