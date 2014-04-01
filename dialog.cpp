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

using namespace std;
Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    timer = new QTimer();
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
    simulate(start, end, &Controllers::kgpkubs);
    ui->horizontalSlider->setRange(0, NUMTICKS-1);
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(onCurIdxChanged(int)));
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    functions.push_back(make_pair("PolarBased", &Controllers::PolarBased));
    functions.push_back(make_pair("kgpkubs", &Controllers::kgpkubs));
    functions.push_back(make_pair("CMU", &Controllers::CMU));
    functions.push_back(make_pair("PController", &Controllers::PController));
    for(unsigned int i = 0; i < functions.size(); i++) {
        ui->simCombo->addItem(functions[i].first);
    }
    onCurIdxChanged(0);
}

Dialog::~Dialog()
{
    delete ui;
}



int Dialog::simulate(Pose startPose, Pose endPose, FType func)
{
    poses[0] = startPose;
    //simulating behaviour for all ticks at once
    int endFlag = 0, timeMs = 0;
    for(int i=1; i < NUMTICKS; i++)
    {
        poses[i] = poses[i-1];
        int vl, vr;
        (*func)(poses[i], endPose, vl, vr);
        vls[i-1] = vl;
        vrs[i-1] = vr;
        if(dist(poses[i], endPose) < 40 && !endFlag) {
            timeMs = i*timeLCMs;
            endFlag = 1;
        }
        poses[i].update(vl, vr, timeLC);
    }
    return timeMs;
}

int Dialog::simulateDelayController(Pose startPose, Pose endPose, FType func)
{
    poses[0] = startPose;
    //simulating behaviour for all ticks at once
    int endFlag = 0, timeMs = 0;
    DelayController dc(func, Pose::numPacketDelay);
    for(int i=1; i < NUMTICKS; i++)
    {
        poses[i] = poses[i-1];
        int vl, vr;
        dc.genControls(poses[i], endPose, vl, vr);
        vls[i-1] = vl;
        vrs[i-1] = vr;
        if(dist(poses[i], endPose) < 40 && !endFlag) {
            timeMs = i*timeLCMs;
            endFlag = 1;
        }
        poses[i].update(vl, vr, timeLC);
    }
    return timeMs;
}


void Dialog::batchSimulation(FType fun)
{
    srand(time(NULL));
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
//            x2 = y2 = theta2 = 0; // doing this for regression with gamma and delta
//            double rho = -rand()%HALF_FIELD_MAXX; // keeping it left side.
//            double angle = rand()/(double)RAND_MAX;
//            angle = normalizeAngle(angle* 2 * PI);
//            angle = 0; // always facing towards final angle.
//            x1 = rho*cos(angle);
//            y1 = rho*sin(angle);
//            theta1 = rand()/(double)RAND_MAX;
//            theta1 = normalizeAngle(theta1*PI);
//            theta1 = PI/4;
//            theta1 = rand()%2?-theta1:theta1;
        }
        Pose start(x1, y1, theta1);
        Pose end(x2, y2, theta2);
        int timeMs = simulateDelayController(start, end, fun);
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
            sprintf(buf, "%g %g %g %d", rho, fabs(gamma), fabs(delta), timeMs);
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


void Dialog::on_simButton_clicked()
{
    Pose start = ui->renderArea->getStartPose();
    Pose end = ui->renderArea->getEndPose();
    FType fun = functions[ui->simCombo->currentIndex()].second;
//    int timeMs = simulate(start, end, fun);
    int timeMs = simulateDelayController(start, end, fun);
    qDebug() << "Bot reached at time tick = " << timeMs/timeLCMs;
    onCurIdxChanged(0);
    on_resetButton_clicked();
}

void Dialog::on_batchButton_clicked()
{
    batchSimulation(functions[ui->simCombo->currentIndex()].second);
}


void Dialog::regression(vector<RegData> func)
{
    int n = func.size();
    gsl_matrix *X = gsl_matrix_calloc(n, 2);
    gsl_vector *Y = gsl_vector_alloc(n);
    gsl_vector *beta = gsl_vector_alloc(2);

    for (int i = 0; i < n; i++) {
        gsl_vector_set(Y, i, func[i].timeMs);

        gsl_matrix_set(X, i, 0, func[i].rho);
        gsl_matrix_set(X, i, 1, func[i].gamma);
//        gsl_matrix_set(X, i, 1, func[i].delta);
    }

    double chisq;
    gsl_matrix *cov = gsl_matrix_alloc(2, 2);
    gsl_multifit_linear_workspace * wspc = gsl_multifit_linear_alloc(n, 2);
    gsl_multifit_linear(X, Y, beta, cov, &chisq, wspc);
    qDebug() << "Beta = " << gsl_vector_get(beta, 0) << ", " << gsl_vector_get(beta, 1);// << ", " << gsl_vector_get(beta, 2);

    gsl_matrix_free(X);
    gsl_matrix_free(cov);
    gsl_vector_free(Y);
    gsl_vector_free(beta);
    gsl_multifit_linear_free(wspc);
}
