#include "dialog.h"
#include "ui_dialog.h"
#include "geometry.h"
#include <stdio.h>
#include <assert.h>
#include <QTimer>
#include <QDebug>
#include <algorithm>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_multifit.h>
#include <limits.h>
#include <limits>
#include <unistd.h>
#include "visionworker.h"
#include "vision-velocity.hpp"
#include "logging.hpp"
#include "trajectory-drawing.hpp"
#include "trajectory-generators.hpp"
#include <fstream>
#include <functional>

using namespace std;
// NOTE(arpit): PREDICTION_PACKET_DELAY is NOT used in simulation. It is used to predict bot position in actual run,
// as well as the algoController delay
static const int PREDICTION_PACKET_DELAY = 4;
// bot used for testing (non-sim)
static const int BOT_ID_TESTING = 0;
Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    traj = NULL;
    std::function<void (void)> func = []() {qDebug() << "hello world";};
    algoController = NULL;
    srand(time(NULL));
    ui->setupUi(this);
    timer = new QTimer();
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
    sim.simulate(start, end, &Controllers::kgpkubs, 0, 0);
//    simulate(start, end, &Controllers::kgpkubs, 0, 0);
    ui->horizontalSlider->setRange(0, NUMTICKS-1);
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(onCurIdxChanged(int)));
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    functions.push_back(make_pair("PolarBidirectional", &Controllers::PolarBidirectional));
    functions.push_back(make_pair("PolarBased", &Controllers::PolarBased));
    functions.push_back(make_pair("kgpkubs", &Controllers::kgpkubs));
    functions.push_back(make_pair("CMU", &Controllers::CMU));
    functions.push_back(make_pair("PController", &Controllers::PController));
    functions.push_back(make_pair("DynamicWindow", &Controllers::DynamicWindow));
    for(unsigned int i = 0; i < functions.size(); i++) {
        ui->simCombo->addItem(functions[i].first);
    }

    ui->receiveDataTextEdit->setReadOnly(true);
    beliefStateSh = new BeliefState;
    bsMutex = new QMutex;
    visionThread = new QThread;
    vw = new VisionWorker;
    vw->setup(visionThread, beliefStateSh, bsMutex, true);
    vw->moveToThread(visionThread);
    visionThread->start();
    ui->firaRenderArea->beliefStateSh = beliefStateSh;
    ui->firaRenderArea->bsMutex = bsMutex;
    connect(vw, SIGNAL(newData()), ui->firaRenderArea, SLOT(update()));
    // Removing this, i think it slows down everything.
//    connect(vw, SIGNAL(newData()), this, SLOT(onNewData()));
    algoTimer = new QTimer();
    connect(algoTimer, SIGNAL(timeout()), this, SLOT(onAlgoTimeout()));
    if(!comm.Open("/dev/ttyUSB0", 38400)) {
        qDebug() << "Could not open comm port!";
    } else {
        qDebug() << "Connected.";
    }
    onCurIdxChanged(0);
    counter = 0;
    sendDataMutex = new QMutex;
}

Dialog::~Dialog()
{
    delete ui;
}






void Dialog::on_startButton_clicked()
{
    timer->start(timeLCMs);
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

void Dialog::on_horizontalSlider_sliderMoved(int )
{
    timer->stop();
}

void Dialog::onCurIdxChanged(int idx)
{
    ui->renderArea->changePose(sim.getPoses(idx));
    MiscData m = sim.getMiscData(idx);
    // lets print for traj sim
//    Pose s = sim.getPoses(idx);
    qDebug() << idx << ". " << "vl, vr = " << sim.getVls(idx) << ", " << sim.getVrs(idx) << ", vl_calc, vr_calc = " <<
                sim.getVls_calc(idx) << ", " << sim.getVrs_calc(idx) << "v_ref, omega_ref = " << m.v_ref << ", " << m.omega_ref << ", "
             << "v1, v2 = " << m.v1 << ", " << m.v2 << "time = " << m.t << "v, w = " << m.v << m.w
             << "vl, vr (in miscdata) = " << m.vl << m.vr;
//    Pose e = ui->renderArea->getEndPose();
    // some debug prints:
//    Vector2D<int> initial(s.x()-e.x(), s.y()-e.y());
//    Vector2D<int> final(0, 0);
//    double theta = normalizeAngle(s.theta() - e.theta());
//    double rho = sqrt(initial.x*initial.x + initial.y*initial.y);
//    double gamma = normalizeAngle(atan2(initial.y, initial.x) - theta + PI);
//    double delta = normalizeAngle(gamma + theta);
//    qDebug() << idx <<". "<< "vl, vr = " << vls[idx] << ", " << vrs[idx] << ", vl_calc, vr_calc = " <<
//                vls_calc[idx] << ", " << vrs_calc[idx] << ", k = " << miscData[idx].k << ", v_curve = " << miscData[idx].v_curve
//             << "finalSpeed = " << miscData[idx].finalSpeed << ", rangeMin = " << miscData[idx].rangeMin << ", rangemax = " << miscData[idx].rangeMax;
                //", rho = " << rho << ", gamma = " << gamma << ", delta = " << delta;
//    qDebug() << "Pose: " << poses[idx].x() << ", " << poses[idx].y() << ", " << poses[idx].theta()*180/PI;
}

void Dialog::onTimeout()
{
    int idx = ui->horizontalSlider->value();
    idx++;
    if(idx >= NUMTICKS) {
        timer->stop();
        return;
    }
    if(idx < 0 || idx >= NUMTICKS) {
        qDebug() << "Error! idx = " << idx << " and is out of range!";
        return;
    }    
    ui->horizontalSlider->setValue(idx);
}

void Dialog::onAlgoTimeout()
{    
    bsMutex->lock();
    BeliefState bs = *beliefStateSh;
    bsMutex->unlock();
    Pose start(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]);
    Pose end = ui->firaRenderArea->getEndPose();
    int vl, vr;
    // NOTE: set finalvel!!
    algoController->genControls(start, end, vl, vr, FINAL_VEL);
    predictedPoseQ.push(algoController->getPredictedPose(start));
    // getPredictedPose gives the predicted pose of the robot after PREDICTION_PACKET_DELAY ticks from now. We need to display what our
    // prediction was PREDICTION_PACKET_DELAY ticks ago (i.e. what our prediction was for now).

    ui->firaRenderArea->predictedPose = predictedPoseQ.front();
    predictedPoseQ.pop();
    assert(vl <= 120 && vl >= -120);
    assert(vr <= 120 && vr >= -120);
    char buf[12];
    for (int i = 0; i < 12; i++)
        buf[i] = 0;
    buf[0] = 127; // doesnt matter
    // NOTE: testing, remove these 2 lines pls
//    vl = 80;
//    vr = 80;

    buf[BOT_ID_TESTING*2 + 1] = vl;
    buf[BOT_ID_TESTING*2 + 2] = vr;
    buf[11] = (++counter)%100;
    qDebug() << "sending: " << vl << vr << counter%100 << ", packets sent = " << counter ;
    sendDataMutex->lock();  
    comm.Write(buf, 12);
    sendDataMutex->unlock();

    // store data in sysData
    sysData.push_back(Logging::populateSystemData(counter%100, vl, vr, bs, BOT_ID_TESTING));
}

void Dialog::onNewData()
{
    bsMutex->lock();
    BeliefState bs = *beliefStateSh;
    bsMutex->unlock();
    // print ball pos
    if (bs.ballIsPresent)
        ui->ballPosLabel->setText(QString("Ball: %1, %2").arg(QString::number(bs.ballX), QString::number(bs.ballY)));
    else
        ui->ballPosLabel->setText("Ball: -");
}


void Dialog::on_simButton_clicked()
{
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
    FType fun = functions[ui->simCombo->currentIndex()].second;    
    double timeMs = sim.simulate(start, end, fun, 0, 0);
    qDebug() << "Bot reached at time tick = " << timeMs/timeLCMs;
    onCurIdxChanged(0);
    on_resetButton_clicked();
}

void Dialog::on_batchButton_clicked()
{
    ui->textEdit->append(sim.batchSimulation(functions[ui->simCombo->currentIndex()].second));
//    qDebug() << "Fitness = " << fitnessFunction(0.05, 20, 5);
}



void Dialog::on_startSending_clicked()
{
    FType fun = functions[ui->simCombo->currentIndex()].second;
    // NOTE: using the trajectory controller for actual bot!
//    algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
    algoController = new ControllerWrapper(fun, 0, 0, PREDICTION_PACKET_DELAY);
    while(!predictedPoseQ.empty())
        predictedPoseQ.pop();
    bsMutex->lock();
    BeliefState bs = *beliefStateSh;
    bsMutex->unlock();
    for (int i = 0; i < PREDICTION_PACKET_DELAY; i++) {
        predictedPoseQ.push(Pose(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]));
    }
    algoTimer->start(timeLCMs);
}

void Dialog::on_stopSending_clicked()
{
    algoTimer->stop();
    char buf[12];
    buf[0] = 127; // doesnt matter;
    for (int i = 1; i < 11; i++)
        buf[i] = 0;
    buf[11] = (++counter)%100; // timestamp
    sendDataMutex->lock();
    usleep(timeLCMs * 1000);
    comm.Write(buf, 12);
    sendDataMutex->unlock();
    // store data in sysData
    bsMutex->lock();
    BeliefState bs = *beliefStateSh;
    bsMutex->unlock();
    sysData.push_back(Logging::populateSystemData(counter%100, 0, 0, bs, BOT_ID_TESTING));
    qDebug() << "sending: 0 0 0, packets sent = " << counter;
    if(algoController) {
        delete algoController;
        algoController = NULL;
    }
}

// temp function for usage here only
void Dialog::readDataAndAppendToLog() {
    sendDataMutex->lock();
    char buf[2];
    buf[0] = 122;
    buf[1] = BOT_ID_TESTING;
    comm.Write(buf, 2);
    sendDataMutex->unlock();
    // int packetSize = 7;
    char syncByte = 122;
    char endByte = 123;
    bool ok;
    int parity = 0;
    char ts = 0, botid = 0, vl_target = 0, vr_target = 0, vl = 0, vr = 0;
    int upperLimit = 8*300; // don't want to get stuck reading.
    int maxMisreadsAllowed = 100;
    int numMisreads = 0;
    int byteCounter = 0;
    while (byteCounter++ < upperLimit && numMisreads < maxMisreadsAllowed) {
        char b = comm.ReadByteTimeout(20, ok);
        if (!ok) {
            qDebug() << "no ok read!";
            numMisreads++;
            continue;
        }
        if (b == endByte)
            break;
        switch(parity) {
        case 0:
            if (b == syncByte)
                parity++;
            break;
        case 1:
            botid = b; parity++; break;
        case 2:
            ts = b; parity++; break;
        case 3:
            vl_target = b; parity++; break;
        case 4:
            vr_target = b; parity++; break;
        case 5:
            vl = b; parity++; break;
        case 6:
            vr = b; parity = 0;
            char buf[100];
            sprintf(buf, "%d:\t\t(%d, %d) ->\t\t(%d, %d)\n", ts, vl_target, vr_target, vl, vr);
            ui->receiveDataTextEdit->insertPlainText(QString(buf));
            recvData.push_back(Logging::populateReceivedData(botid, ts, vl_target, vr_target, vl, vr));
            break;
        }
    }
    if (byteCounter >= upperLimit) {
        qDebug() << "Read more than allowed number of bytes in receiving data.";
    }
    if (numMisreads > maxMisreadsAllowed ) {
        qDebug() << "Exceeded maximum allowed misreads in receiving data.";
    }
//    if (recvData.size())
//        recvData.pop_back();  // removing the stop command packet.

    Logging::Log log_temp = Logging::mergeSysRecvLists(sysData, recvData);
    for (int i = 0; i < log_temp.data_size(); i++) {
        *log.add_data() = log_temp.data(i);
    }

    // clear logging structs
    sysData.clear();
    recvData.clear();
}

void Dialog::on_receiveButton_clicked()
{
    readDataAndAppendToLog();
}

void Dialog::on_clearButton_clicked()
{
    log.clear_data();
    ui->receiveDataTextEdit->clear();    
}

void Dialog::on_writeLogButton_clicked()
{
    fstream output(ui->logFileLineEdit->text().toStdString().c_str(),  ios::out | ios::trunc | ios::binary);
    if (!output.is_open()) {
        qDebug() << "Failed to open output file " << ui->logFileLineEdit->text();
    }
    if (!log.SerializeToOstream(&output))
        qDebug() << "Failed to write log to file.";
    qDebug() << "Successfully wrote log to " << ui->logFileLineEdit->text();
    output.close();
    log.clear_data();
    sysData.clear();
    recvData.clear();
}

void Dialog::on_trajCheckbox_toggled(bool checked)
{
    ui->renderArea->toggleTrajectory(checked);
}

void Dialog::on_trajButton_clicked()
{
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
    FType fun = functions[ui->simCombo->currentIndex()].second;
    ui->renderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(fun, start, 0, 0, end, FINAL_VEL,
                                                                       FINAL_VEL, 4000, timeLCMs));
    if (!ui->trajCheckbox->isEnabled()) {
        ui->trajCheckbox->setEnabled(true);
        ui->trajCheckbox->setChecked(true);
    }    
}

void Dialog::on_traj2Button_clicked()
{
    // not using this right now!
//    bsMutex->lock();
//    BeliefState bs = *beliefStateSh;
//    bsMutex->unlock();
//    Pose start(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]);
//    Pose end = ui->firaRenderArea->getEndPose();
//    FType fun = functions[ui->simCombo->currentIndex()].second;
//    ui->firaRenderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(fun, start, 0, 0, end, FINAL_VEL,
//                                                                       FINAL_VEL, 4000, timeLCMs));
//    ui->firaRenderArea->toggleTrajectory(true);
}

void Dialog::on_circleTrajButton_clicked()
{
    using namespace TrajectoryGenerators;
    double x = ui->xCircle->text().toDouble();
    double y = ui->yCircle->text().toDouble();
    double startTheta = ui->thetaCircle->text().toDouble();
    double r = ui->rCircle->text().toDouble();
    double f = ui->fCircle->text().toDouble();
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
//    traj = circleGenerator(x,y,r,startTheta,f);
    if (traj)
        delete traj;
//    traj = quinticBezierSplineGenerator(start, end, 0, 0, 40, 70);
//    traj = cubic(start, end, 0, 0, 40, 70);
    traj = cubic2CP(start, end, 0, 0, 40, 70);
    ui->renderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(*traj, 4000, timeLCMs));
    if (ui->trajSimButton->isEnabled() == false)
        ui->trajSimButton->setEnabled(true);
    if (!ui->trajCheckbox->isEnabled()) {
        ui->trajCheckbox->setEnabled(true);
        ui->trajCheckbox->setChecked(true);
    }
    ui->renderArea->toggleTrajectory(true);

    ui->firaRenderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(*traj, 4000, timeLCMs));
    ui->firaRenderArea->toggleTrajectory(true);
}

void Dialog::on_trajSimButton_clicked()
{    
    Pose start = ui->renderArea->getStartPose();
    sim.simulate(start, traj, 0, 0, false);
    onCurIdxChanged(0);
    on_resetButton_clicked();
}
