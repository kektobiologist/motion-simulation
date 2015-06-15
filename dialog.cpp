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
#include "defender.hpp"
#include "goalie.hpp"
#include "trajectory-generators.hpp"
#include "ballinterception.hpp"
#include <fstream>
#include <functional>
#include "tests.hpp"

using namespace std;
// NOTE(arpit): PREDICTION_PACKET_DELAY is NOT used in simulation. It is used to predict bot position in actual run,
// as well as the algoController delay
static const int PREDICTION_PACKET_DELAY = 4;
// bot used for testing (non-sim)
static const int BOT_ID_TESTING = 2;
static bool USING_INTERCEPTION = false;
int flag = 0;

RenderArea *gRenderArea = NULL;
Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    direction = true;
    traj = NULL;
    std::function<void (void)> func = []() {qDebug() << "hello world";};
    algoController = NULL;
    srand(time(NULL));
    ui->setupUi(this);
    timer = new QTimer();
    gRenderArea = ui->renderArea;
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
    sim.simulate(start, end, &Controllers::kgpkubs, 0, 0);
    simsc.simulate(start, end, &Controllers::kgpkubs, 0, 0);
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
    vw->setup(visionThread, beliefStateSh, bsMutex, false);
    vw->moveToThread(visionThread);
    visionThread->start();
    ui->firaRenderArea->beliefStateSh = beliefStateSh;
    ui->firaRenderArea->bsMutex = bsMutex;
    connect(vw, SIGNAL(newData()), ui->firaRenderArea, SLOT(update()));
    // Removing this, i think it slows down everything.
//    connect(vw, SIGNAL(newData()), this, SLOT(onNewData()));
    algoTimer = new QTimer();
    connect(algoTimer, SIGNAL(timeout()), this, SLOT(onAlgoTimeout()));
    QTimer *printTimer = new QTimer();
    printTimer->setSingleShot(false);
//    printTimer->start(500);
    connect(printTimer, SIGNAL(timeout()), this, SLOT(onNewData()));
    if(!comm.Open("/dev/ttyUSB0", 38400)) {
        qDebug() << "Could not open comm port!";
    } else {
        qDebug() << "Connected.";
    }
    onCurIdxChanged(0);
    counter = 0;
    sendDataMutex = new QMutex;    double vx = 0, vy = 0;
    for (int i = 0; i < 4; i++) {
        ballPoses.push(Vector2D<double>(0, 0));
        ballVels.push(Vector2D<double>(0, 0));
    }
}

Dialog::~Dialog()
{
    delete ui;
}

bool Dialog::isFrontDirected(Pose botPos, Pose endPos) {
    int r = 10;
    double cosTheta = cos(botPos.theta());
    double sinTheta = sin(botPos.theta());
    double testx = botPos.x() + r * cosTheta;
    double testy = botPos.y() + r * sinTheta;
    double v1 = testx / tan(botPos.theta()) + testy - botPos.x() / tan(botPos.theta()) - botPos.y();
    double v2 = endPos.x() / tan(botPos.theta()) + endPos.y() - botPos.x() / tan(botPos.theta()) - botPos.y();
    return ((v1 * v2) >= 0) ? true : false;
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
    if (idx >= 40 && flag==0) {
        flag=1;
        on_splineChangeBtn_clicked();
    }
    if(idx < 40){
        qDebug() <<  "\n This is the original trajectory";
        ui->renderArea->changePose(sim.getPoses(idx));
        MiscData m = sim.getMiscData(idx);
        qDebug() << idx << ". " << "vl, vr = " << sim.getVls(idx) << ", " << sim.getVrs(idx) << ", vl_calc, vr_calc = " <<
                    sim.getVls_calc(idx) << ", " << sim.getVrs_calc(idx) << "v_ref, omega_ref = " << m.v_ref << ", " << m.omega_ref << ", "
                 << "v1, v2 = " << m.v1 << ", " << m.v2 << "time = " << m.t << "v, w = " << m.v << m.w
                 << "vl, vr (in miscdata) = " << m.vl << m.vr << "vl_ref, vr_ref = " << m.vl_ref << m.vr_ref;
    }
    else{
        qDebug() <<  "\n\t\t This is the trajectory after changing";
        ui->renderArea->changePose(simsc.getPoses(idx-40));
        MiscData m = simsc.getMiscData(idx-40);
        qDebug() << idx-40 << ". " << "vl, vr = " << simsc.getVls(idx-40) << ", " << simsc.getVrs(idx-40) << ", vl_calc, vr_calc = " <<
                    simsc.getVls_calc(idx-40) << ", " << simsc.getVrs_calc(idx-40) << "v_ref, omega_ref = " << m.v_ref << ", " << m.omega_ref << ", "
                 << "v1, v2 = " << m.v1 << ", " << m.v2 << "time = " << m.t << "v, w = " << m.v << m.w
                 << "vl, vr (in miscdata) = " << m.vl << m.vr << "vl_ref, vr_ref = " << m.vl_ref << m.vr_ref;
    }

    // lets print for traj sim
//    Pose s = sim.getPoses(idx);

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

    if(!direction){
        start = Pose(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]-PI);
    }

    Vector2D<double> ballPos(bs.ballX, bs.ballY);
    double dist = BallInterception::getBotBallDist(start, ballPos);
    //TDefend tdef;
    //Pose end = tdef.execute(&bs, BOT_ID_TESTING);
    int vl, vr;
    if (USING_INTERCEPTION) {
//        assert(0);
        // if bot is close to end point, then make a new trajectory that leads to goal!

        if(traj){
            SplineTrajectory *st = dynamic_cast<SplineTrajectory*>(traj);
            double dt = st->totalTime() - algoController->getCurrentTimeS();
            //qDebug() << start.x() << " " << start.y() << endl;
          //  qDebug() << "dt = " << dt << "st->totalTime() = " << st->totalTime();
        }
        if (dist <= 1.5*BOT_RADIUS) { //dt = 0.21
            USING_INTERCEPTION = false;
            // make a new trajectory
            if (traj)
                delete traj;
            using namespace TrajectoryGenerators;
            Vector2D<double> goalCentre(HALF_FIELD_MAXX, 0);
            double endTheta = atan2(goalCentre.y - start.y(), goalCentre.x - start.x());
            Pose endPose(goalCentre.x, goalCentre.y, endTheta);
            Pose cp1(bs.ballX, bs.ballY, 0);
            vector<Pose> midPoints;
//            midPoints.push_back(cp1);
            traj = cubic(start, endPose, bs.homeVl[BOT_ID_TESTING], bs.homeVr[BOT_ID_TESTING], 70, 70, midPoints);
            ui->firaRenderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(*traj, 4000, timeLCMs));
            if (ui->trajSimButton->isEnabled() == false)
                ui->trajSimButton->setEnabled(true);
            if (!ui->trajCheckbox->isEnabled()) {
                ui->trajCheckbox->setEnabled(true);
                ui->trajCheckbox->setChecked(true);
            }
            ui->renderArea->toggleTrajectory(true);

            ui->firaRenderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(*traj, 4000, timeLCMs));
            ui->firaRenderArea->toggleTrajectory(true);

            algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
            //algoController = new ControllerWrapper(traj, bs.homeVl[BOT_ID_TESTING], bs.homeVr[BOT_ID_TESTING], PREDICTION_PACKET_DELAY);
        }
    }
    // NOTE: set finalvel!!
    algoController->genControls(start, end, vl, vr, FINAL_VEL);
    predictedPoseQ.push_back(algoController->getPredictedPose(start));
    // getPredictedPose gives the predicted pose of the robot after PREDICTION_PACKET_DELAY ticks from now. We need to display what our
    // prediction was PREDICTION_PACKET_DELAY ticks ago (i.e. what our prediction was for now).

    ui->firaRenderArea->predictedPose = predictedPoseQ.front();
    predictedPoseQ.pop_front();
    assert(vl <= 120 && vl >= -120);
    assert(vr <= 120 && vr >= -120);
    char buf[12];
    for (int i = 0; i < 12; i++)
        buf[i] = 0;
    buf[0] = 126; // doesnt matter
    // NOTE: testing, remove these 2 lines pls
//    vl = 80;
//    vr = 80;

    if (direction) {
        buf[BOT_ID_TESTING*2 + 1] = vl;
        buf[BOT_ID_TESTING*2 + 2] = vr;
    } else {
        buf[BOT_ID_TESTING*2 + 1] = -vr;
        buf[BOT_ID_TESTING*2 + 2] = -vl;
    }
    getVel.x = vl;
    getVel.y = vr;
    buf[11] = (++counter)%100;
    qDebug() << "sending: " << vl << vr << counter%100 << ", packets sent = " << counter ;
    sendDataMutex->lock();
    comm.Write(buf, 12);
    sendDataMutex->unlock();
    if (counter > 50 && flag==0) {
        qDebug() << "Changing trajectoiry ";
        counter=0;flag=1;
        on_traj2Button_clicked();
        on_startSending_clicked();
    }
   // else  // store data in sysData
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
    // print bot velocitys
    if (bs.homeIsPresent[BOT_ID_TESTING]) {
        ui->ballPosLabel->setText(QString("bot: %1, %2").arg(QString::number(bs.homeVl[BOT_ID_TESTING]),
                                                             QString::number(bs.homeVr[BOT_ID_TESTING])));
    } else {
        ui->ballPosLabel->setText("Bot: -");
    }
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
    //algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
   // algoController = new ControllerWrapper(fun, 0, 0, PREDICTION_PACKET_DELAY);
    while(!predictedPoseQ.empty())
        predictedPoseQ.pop_front();
    bsMutex->lock();
    BeliefState bs = *beliefStateSh;
    bsMutex->unlock();
    algoController = new ControllerWrapper(traj, bs.homeVl[BOT_ID_TESTING], bs.homeVr[BOT_ID_TESTING], PREDICTION_PACKET_DELAY);
    for (int i = 0; i < PREDICTION_PACKET_DELAY; i++) {
        predictedPoseQ.push_back(Pose(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]));
    }
    algoTimer->start(timeLCMs);
}

void Dialog::on_stopSending_clicked()
{
    algoTimer->stop();
    char buf[12];
    buf[0] = 126; // doesnt matter;
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
    USING_INTERCEPTION = false;
    // not using this right now!
//    bsMutex->lock();
//    BeliefState bs = *beliefStateSh;
//    bsMutex->unlock();
//    Pose start(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]);
//    //qDebug() << start.x() << start.y() << start.theta() << "\n";
//    Pose end = ui->firaRenderArea->getEndPose();
//    FType fun = functions[ui->simCombo->currentIndex()].second;
//    ui->firaRenderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(fun, start, 0, 0, end, FINAL_VEL,
//                                                                       FINAL_VEL, 4000, timeLCMs));
//    ui->firaRenderArea->toggleTrajectory(true);

    bsMutex->lock();
    BeliefState bs = *beliefStateSh;
    bsMutex->unlock();
    using namespace TrajectoryGenerators;
    Pose start(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]);
    Pose start2(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]-PI);
    Pose end = ui->firaRenderArea->getEndPose();
    if (traj)
        delete traj;
//    traj = quinticBezierSplineGenerator(start, end, 0, 0, 0, 0);
    //direction = isFrontDirected(start, end) ;
    if(direction){
        if(!flag)
            traj = cubic(start, end, 0, 0, 0, 0);
        else
            traj = cubic(start, end, bs.homeVl[BOT_ID_TESTING], bs.homeVr[BOT_ID_TESTING], 0, 0);
    }
    else {
        if(!flag)
            traj = cubic(start2, end, 0, 0, 0, 0);
        else
            traj = cubic(start2, end, bs.homeVl[BOT_ID_TESTING], bs.homeVr[BOT_ID_TESTING], 0, 0);
    }
    flag=0;
    ui->firaRenderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(*traj, 4000, timeLCMs));
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

void Dialog::on_circleTrajButton_clicked()
{
    // adding test code here
    //Tests::arclengthParam_test(100);
    //return;
    using namespace TrajectoryGenerators;
    double x = ui->xCircle->text().toDouble();
    double y = ui->yCircle->text().toDouble();
    double startTheta = ui->thetaCircle->text().toDouble();
    double r1 = ui->rCircle1->text().toDouble();
    double r2 = ui->rCircle2->text().toDouble();
    double f = ui->fCircle->text().toDouble();
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
    if (traj)
        delete traj;

      // traj = circleGenerator(x,y,r1,startTheta,f);
//    traj = quinticBezierSplineGenerator(start, end, 0, 0, 40, 70);aj = cubic2CP(start, end, 0, 0, 40, 70);

    //traj = cubic(start, end, 0, 0, 40, 70);
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

void Dialog::on_splineChangeBtn_clicked() {
    using namespace TrajectoryGenerators;
    int idx = ui->horizontalSlider->value();
    Pose start;
    if (idx == 0)
        start = ui->renderArea->getStartPose();
    else
        start = sim.getPoses(idx);
    Pose end = ui->renderArea->getEndPose();
    if (traj)
        delete traj;

    if (!idx)
        traj = cubic(start, end, 0, 0, 70, 70);
    else
        traj = cubic(start, end, sim.getVls(idx), sim.getVrs(idx), 70, 70);

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
    on_trajSimButton_clicked();
    //on_startButton_clicked();
}

void Dialog::on_trajSimButton_clicked()
{
    Pose start;
    int idx = ui->horizontalSlider->value();
    if (idx == 0)
        start = ui->renderArea->getStartPose();
    else
        start = sim.getPoses(idx);
    if (idx == 0)
        sim.simulate(start, traj, 0, 0, false);
    else
        simsc.simulate(start, traj, sim.getVls(idx), sim.getVrs(idx), false);
    //onCurIdxChanged(0);
    //on_resetButton_clicked();
}

void Dialog::on_interceptionButton_clicked()
{
    bsMutex->lock();
    BeliefState bs = *beliefStateSh;
    bsMutex->unlock();
    USING_INTERCEPTION = true;
    using namespace TrajectoryGenerators;
    double vx = 0., vy = 0.;

    if(traj){

        SplineTrajectory *bi_traj = dynamic_cast<SplineTrajectory*>(traj);
        vector<VelocityProfiling::ProfileDatapoint> profile = bi_traj->getProfile();

        double t = 70 * 0.016;
        int x = 0;
        for (int i = 0; i < 1000; i++) {
            if (profile[i].t > t) {
                x = i;
                break;
            }
        }
        for (int i = 0; i < 1; i++) {
            vx += profile[x+i].v * cos(bs.homeTheta[BOT_ID_TESTING]);
            vy += profile[x+i].v * sin(bs.homeTheta[BOT_ID_TESTING]); //try using PredictedPoseQ for theta.
        }

        qDebug() << "Changing SPline Pos";
        //delete traj;
    }
    vx = (bs.homeVl[BOT_ID_TESTING] + bs.homeVr[BOT_ID_TESTING]) * cos(bs.homeTheta[BOT_ID_TESTING]) / 2;
    vy = (bs.homeVl[BOT_ID_TESTING] + bs.homeVr[BOT_ID_TESTING]) * sin(bs.homeTheta[BOT_ID_TESTING]) / 2;
    qDebug() << bs.ballVx << "Dasda " << bs.ballVy << endl;
    Pose start(bs.homeX[BOT_ID_TESTING] + 0.016 * vx, bs.homeY[BOT_ID_TESTING] + 0.016 * vy, bs.homeTheta[BOT_ID_TESTING]);
    Vector2D<double> ballPos(bs.ballX, bs.ballY);
    Vector2D<double> ballVel(bs.ballVx, bs.ballVy);
    Vector2D<double> botVel(bs.homeVl[BOT_ID_TESTING], bs.homeVr[BOT_ID_TESTING]);
    traj = BallInterception::getIntTraj(start, ballPos, ballVel, botVel);
    ui->firaRenderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(*traj, 4000, timeLCMs));
    if (ui->trajSimButton->isEnabled() == false)
        ui->trajSimButton->setEnabled(true);
    if (!ui->trajCheckbox->isEnabled()) {
        ui->trajCheckbox->setEnabled(true);
        ui->trajCheckbox->setChecked(true);
    }
    ui->renderArea->toggleTrajectory(true);

    ui->firaRenderArea->setTrajectory(TrajectoryDrawing::getTrajectoryPath(*traj, 4000, timeLCMs));
    ui->firaRenderArea->toggleTrajectory(true);
   on_startSending_clicked();
}
