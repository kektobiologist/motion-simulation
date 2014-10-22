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
#include "visionworker.h"
using namespace std;
// NOTE(arpit): PREDICTION_PACKET_DELAY is NOT used in simulation. It is used to predict bot position in actual run.
static const int PREDICTION_PACKET_DELAY = 0;
// bot used for testing (non-sim)
static const int BOT_ID_TESTING = 0;
Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    algoController = NULL;
    srand(time(NULL));
    ui->setupUi(this);
    timer = new QTimer();
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
    simulate(start, end, &Controllers::kgpkubs);
    ui->horizontalSlider->setRange(0, NUMTICKS-1);
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(onCurIdxChanged(int)));
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    functions.push_back(make_pair("PolarBidirectional", &Controllers::PolarBidirectional));
    functions.push_back(make_pair("PolarBased", &Controllers::PolarBased));
    functions.push_back(make_pair("kgpkubs", &Controllers::kgpkubs));
    functions.push_back(make_pair("CMU", &Controllers::CMU));
    functions.push_back(make_pair("PController", &Controllers::PController));
    for(unsigned int i = 0; i < functions.size(); i++) {
        ui->simCombo->addItem(functions[i].first);
    }

    ui->receiveDataTextEdit->setReadOnly(true);
    beliefStateSh = new BeliefState;
    bsMutex = new QMutex;
    visionThread = new QThread;
    vw = new VisionWorker;
    vw->setup(visionThread, beliefStateSh, bsMutex);
    vw->moveToThread(visionThread);
    visionThread->start();
    ui->firaRenderArea->beliefStateSh = beliefStateSh;
    ui->firaRenderArea->bsMutex = bsMutex;
    connect(vw, SIGNAL(newData()), ui->firaRenderArea, SLOT(update()));
    algoTimer = new QTimer();
    connect(algoTimer, SIGNAL(timeout()), this, SLOT(onAlgoTimeout()));
    if(!comm.Open("/dev/ttyUSB0", 38400)) {
        qDebug() << "Could not open comm port!";
    } else {
        qDebug() << "Connected.";
    }
    onCurIdxChanged(0);
    counter = 0;
}

Dialog::~Dialog()
{
    delete ui;
}





double Dialog::simulate(Pose startPose, Pose endPose, FType func, bool isBatch)
{
    poses[0] = startPose;
    //simulating behaviour for all ticks at once
    int endFlag = 0;
    double timeMs = std::numeric_limits<double>::max();
    ControllerWrapper dc(func, 0);
    for(int i=1; i < NUMTICKS; i++)
    {
        poses[i] = poses[i-1];
        int vl, vr;
        dc.genControls(poses[i], endPose, vl, vr);
        vls[i-1] = vl;
        vrs[i-1] = vr;
        poses[i].update(vl, vr, timeLC);
        if(dist(poses[i], endPose) < 40 && !endFlag) {
            timeMs = i*timeLCMs;
            endFlag = 1;
            if(isBatch)
                break;
        }
    }
    return timeMs;
}


void Dialog::batchSimulation(FType fun)
{

    vector<RegData> func; // (dist,theta) maps to timeMs
    for(int i = 0; i < 300; i++) {
        int x1 = rand()%HALF_FIELD_MAXX;
        x1 = rand()%2?-x1:x1;
        int y1 = rand()%HALF_FIELD_MAXY;
        y1 = rand()%2?-y1:y1;
        double theta1 = rand()/(double)RAND_MAX;
        theta1 = normalizeAngle(theta1 * 2 * PI);
        int x2 = rand()%HALF_FIELD_MAXX;
        x2 = rand()%2?-x2:x2;
        int y2 = rand()%HALF_FIELD_MAXY;
        y2 = rand()%2?-y2:y2;
        double theta2 = rand()/(double)RAND_MAX;
        theta2 = normalizeAngle(theta2 * 2 * PI);
        {
            x2 = y2 = theta2 = 0;
        }
        Pose start(x1, y1, theta1);
        Pose end(x2, y2, theta2);
        double timeMs = simulate(start, end, fun, true);
        {
            // calculate rho, gamma, delta
            Pose s(x1, y1, theta1);
            Pose e(x2, y2, theta2);
            Vector2D<int> initial(s.x()-e.x(), s.y()-e.y());
            double theta = normalizeAngle(s.theta() - e.theta());
            // rotate initial by -e.theta degrees;
            double newx = initial.x * cos(-e.theta()) - initial.y * sin(-e.theta());
            double newy = initial.x * sin(-e.theta()) + initial.y * cos(-e.theta());
            initial = Vector2D<int>(newx, newy);
            double rho = sqrt(initial.x*initial.x + initial.y*initial.y);
            double gamma = normalizeAngle(atan2(initial.y, initial.x) - theta + PI);
            double delta = normalizeAngle(gamma + theta);
            func.push_back(RegData(rho, gamma, delta, timeMs));
            char buf[1000];
            sprintf(buf, "%g %g %g %g", rho, fabs(gamma), fabs(delta), timeMs);
            ui->textEdit->append(buf);
        }
//        sprintf(buf, "Pose (%d, %d, %lf) to (%d, %d, %lf) simulating..", x1, y1, theta1, x2, y2, theta2);
//        if(dist(end, poses[NUMTICKS-1]) > 50 || fabs(normalizeAngle(poses[NUMTICKS-1].theta() - end.theta())) > PI/10) {
//            sprintf(buf, "Did not reach! Distance = %lf", dist(end, poses[NUMTICKS-1]));
//            ui->textEdit->append(buf);
//            ui->renderArea->setStartPose(start);
//            ui->renderArea->setEndPose(end);
//            break;
//        } else {
//            sprintf(buf, "Reached. Distance from end = %lf.", dist(end, poses[NUMTICKS-1]));
//            ui->textEdit->append(buf);
//        }
    }
    regression(func);
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
    ui->renderArea->changePose(poses[idx]);
    Pose s = poses[idx];
    Pose e = ui->renderArea->getEndPose();
    Vector2D<int> initial(s.x()-e.x(), s.y()-e.y());
    Vector2D<int> final(0, 0);
    double theta = normalizeAngle(s.theta() - e.theta());
    double rho = sqrt(initial.x*initial.x + initial.y*initial.y);
    double gamma = normalizeAngle(atan2(initial.y, initial.x) - theta + PI);
    double delta = normalizeAngle(gamma + theta);
    qDebug() << idx <<". "<< "vl, vr = " << vls[idx] << ", " << vrs[idx] << ", rho = " << rho << ", gamma = " << gamma << ", delta = " << delta;
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
    algoController->genControls(start, end, vl, vr);
    // getPredictedPose gives the predicted pose of the robot after PREDICTION_PACKET_DELAY ticks from now. We need to display what our
    // prediction was PREDICTION_PACKET_DELAY ticks ago (i.e. what our prediction was for now).
    predictedPoseQ.push(algoController->getPredictedPose(start));
    ui->firaRenderArea->predictedPose = predictedPoseQ.front();
    predictedPoseQ.pop();
    assert(vl < 120 || -vl < 120);
    assert(vr < 120 || -vr < 120);
    char buf[4];
    buf[0] = 126; // doesnt matter
    buf[1] = vl;
    buf[2] = vr;
    buf[3] = (++counter)%100;
    qDebug() << "sending: " << vl << vr << counter%100 << ", packets sent = " << counter ;
    comm.Write(buf, 4);
}


void Dialog::on_simButton_clicked()
{
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
    FType fun = functions[ui->simCombo->currentIndex()].second;
    double timeMs = simulate(start, end, fun);
    qDebug() << "Bot reached at time tick = " << timeMs/timeLCMs;
    onCurIdxChanged(0);
    on_resetButton_clicked();
}

void Dialog::on_batchButton_clicked()
{
    batchSimulation(functions[ui->simCombo->currentIndex()].second);
//    qDebug() << "Fitness = " << fitnessFunction(0.05, 20, 5);
}


void Dialog::regression(vector<RegData> func)
{
    int n = func.size();
    gsl_matrix *X = gsl_matrix_calloc(n, 2);
    gsl_vector *Y = gsl_vector_alloc(n);
    gsl_vector *beta = gsl_vector_alloc(2);

    for (int i = 0; i < n; i++) {
        gsl_vector_set(Y, i, func[i].timeMs);

//        gsl_matrix_set(X, i, 0, 1);
        gsl_matrix_set(X, i, 0, pow(func[i].rho, 1/2.0));
        gsl_matrix_set(X, i, 1, pow(func[i].rho, 1/3.0));
//        gsl_matrix_set(X, i, 1, func[i].gamma);
//        gsl_matrix_set(X, i, 1, func[i].delta);
    }

    double chisq;
    gsl_matrix *cov = gsl_matrix_alloc(2, 2);
    gsl_multifit_linear_workspace * wspc = gsl_multifit_linear_alloc(n, 2);
    gsl_multifit_linear(X, Y, beta, cov, &chisq, wspc);
    qDebug() << "Beta = " << gsl_vector_get(beta, 0) << ", " << gsl_vector_get(beta, 1) << ", chisq = " << chisq;// << ", " << gsl_vector_get(beta, 2);

    gsl_matrix_free(X);
    gsl_matrix_free(cov);
    gsl_vector_free(Y);
    gsl_vector_free(beta);
    gsl_multifit_linear_free(wspc);
}

double Dialog::fitnessFunction(double k1, double k2, double k3)
{
    srand(time(NULL));
    double val = 0;
    for(int j = 0; j < 300; j++) {
        Pose poses[NUMTICKS];
        int x1 = rand()%HALF_FIELD_MAXX;
        x1 = rand()%2?-x1:x1;
        int y1 = rand()%HALF_FIELD_MAXY;
        y1 = rand()%2?-y1:y1;
        double theta1 = rand()/(double)RAND_MAX;
        theta1 = normalizeAngle(theta1 * 2 * PI);
        int x2 = rand()%HALF_FIELD_MAXX;
        x2 = rand()%2?-x2:x2;
        int y2 = rand()%HALF_FIELD_MAXY;
        y2 = rand()%2?-y2:y2;
        double theta2 = rand()/(double)RAND_MAX;
        theta2 = normalizeAngle(theta2 * 2 * PI);
        Pose start(x1, y1, theta1);
        Pose end(x2, y2, theta2);
        poses[0] = start;
        //simulating behaviour for all ticks at once
        int endFlag = 0;
        double timeMs = std::numeric_limits<double>::max();
        for(int i=1; i < NUMTICKS; i++)
        {
            poses[i] = poses[i-1];
            int vl, vr;
            Controllers::PolarBasedGA(poses[i], end, vl, vr, k1, k2, k3);
            if(dist(poses[i], end) < 40 && !endFlag) {
                timeMs = i*timeLCMs;
                endFlag = 1;
            }
            poses[i].update(vl, vr, timeLC);
        }
        val += timeMs;
    }
    return val;
}

void Dialog::on_startSending_clicked()
{
    algoController = new ControllerWrapper(Controllers::PolarBidirectional, PREDICTION_PACKET_DELAY);
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
    char buf[4];
    buf[0] = 126; // doesnt matter;
    buf[1] = buf[2] = 0;
    buf[3] = 0; // timestamp
    comm.Write(buf, 4);
    qDebug() << "sending: 0 0 0, packets sent = " << ++counter;
    if(algoController) {
        delete algoController;
        algoController = NULL;
    }
}

void Dialog::on_receiveButton_clicked()
{
    comm.WriteByte(122);
    int numPacketsToReceive = 200;
    int packetSize = 5;
    char syncByte = 122;
    bool ok;
    // I'll just read the expected number of times, error handling later.
    int errorCount = 0;
    for (int i = 0; i < numPacketsToReceive; i++) {
        char first = 0;
        while (first != syncByte && errorCount < 100)
            first = comm.ReadByteTimeout(20, ok), errorCount++;
        errorCount--;
        char ts = comm.ReadByteTimeout(20, ok);
        char vl_target = comm.ReadByteTimeout(20, ok);
        char vr_target = comm.ReadByteTimeout(20, ok);
        char vl = comm.ReadByteTimeout(20, ok);
        char vr = comm.ReadByteTimeout(20, ok);
        char buf[100];
        sprintf(buf, "%d: %d:\t\t(%d, %d) ->\t\t(%d, %d)\n", first, ts, vl_target, vr_target, vl, vr);
        ui->receiveDataTextEdit->insertPlainText(QString(buf));
    }
}

void Dialog::on_clearButton_clicked()
{
    ui->receiveDataTextEdit->clear();
}
