#include "dialog.h"
#include "ui_dialog.h"
#include <fstream>
#include <QDebug>
#include <QFileDialog>
#include <QFile>
#include "logging.pb.h"
#include "beliefstate.h"
using namespace std;

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    timer = new QTimer();
    ui->horizontalSlider->setRange(0, 0);
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(onCurIdxChanged(int)));
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::on_loadButton_clicked()
{
    QString fileName = ui->lineEdit->text();
    fstream input(fileName.toAscii(), ios::in | ios::binary);
    if (!input.is_open()) {
        qDebug() << "Could not open log file.";
        return;
    }
    data.clear_data();
    if (!data.ParseFromIstream(&input)) {
        qDebug() << "Could not read log from file.";
        return;
    }
    qDebug() << "Successfully loaded log.";
    ui->horizontalSlider->setRange(0, data.data_size() - 1);
    ui->horizontalSlider->setValue(0);
}

void Dialog::onCurIdxChanged(int idx) {
    BeliefState bs;
    if (idx < 0 || idx >= data.data_size())
        return;
    const Logging::LoggingData &ldata = data.data(idx);
    // set drawing specific stuff in bs, ie home x, y, theta, ispresent
    int botid = ldata.sysdata().id();
    bs.homeX[botid] = ldata.sysdata().pose().x();
    bs.homeY[botid] = ldata.sysdata().pose().y();
    bs.homeTheta[botid] = ldata.sysdata().pose().theta();
    bs.homeIsPresent[botid] = true;
    ui->renderArea->setBeliefState(bs);
    ui->renderArea->update();

    ui->sentVLabel->setText(QString("%1, %2").arg(QString::number(ldata.sysdata().sent().vl()), QString::number(ldata.sysdata().sent().vr())));
    ui->visionVLabel->setText(QString("%1, %2").arg(QString::number(ldata.sysdata().vision().vl()), QString::number(ldata.sysdata().vision().vr())));
    ui->tsSystemLabel->setText(QString::number(ldata.sysdata().ts()));

    if (ldata.has_recvdata()) {
        ui->targetVLabel->setText(QString("%1, %2").arg(QString::number(ldata.recvdata().target().vl()), QString::number(ldata.recvdata().target().vr())));
        ui->ticksVLabel->setText(QString("%1, %2").arg(QString::number(ldata.recvdata().measured().vl()), QString::number(ldata.recvdata().measured().vr())));
        ui->tsReceivedLabel->setText(QString::number(ldata.recvdata().ts()));
    } else {
    ui->targetVLabel->setText(QString("-"));
    ui->ticksVLabel->setText(QString("-"));
    }
//    ui->grou
}

void Dialog::onTimeout() {
    int idx = ui->horizontalSlider->value();
    idx++;
    if(idx >= data.data_size()) {
        timer->stop();
        return;
    }
    if(idx < 0 || idx >= data.data_size()) {
        qDebug() << "Error! idx = " << idx << " and is out of range!";
        return;
    }
    ui->horizontalSlider->setValue(idx);
}

void Dialog::on_startButton_clicked()
{
    timer->start(timeMs);
}

void Dialog::on_pauseButton_clicked()
{
    timer->stop();
}

void Dialog::on_resetButton_clicked()
{
    timer->stop();
    ui->horizontalSlider->setValue(0);
}

void Dialog::on_horizontalSlider_sliderMoved(int position)
{
    timer->stop();
}

void Dialog::on_saveAsButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save Log"), "",
                                                    tr("All Files (*)"));
    if (filename.isEmpty())
        return;
    else {
        fstream output(filename.toStdString().c_str(),  ios::out | ios::trunc | ios::binary);
        if (!output.is_open()) {
            qDebug() << "Failed to open output file " << filename;
        }
        if (!data.SerializeToOstream(&output))
            qDebug() << "Failed to write log to file.";
        qDebug() << "Successfully wrote log to " << filename;
    }
}
