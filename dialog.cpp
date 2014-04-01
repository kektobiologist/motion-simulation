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
    simulate(start, end, &Dialog::kgpkubs);
    ui->horizontalSlider->setRange(0, NUMTICKS-1);
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(onCurIdxChanged(int)));
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    functions.push_back(make_pair("PolarBased", &Dialog::PolarBased));
    functions.push_back(make_pair("kgpkubs", &Dialog::kgpkubs));
    functions.push_back(make_pair("CMU", &Dialog::CMU));
    functions.push_back(make_pair("PController", &Dialog::PController));
    for(unsigned int i = 0; i < functions.size(); i++) {
        ui->simCombo->addItem(functions[i].first);
    }
    onCurIdxChanged(0);
}

Dialog::~Dialog()
{
    delete ui;
}


void Dialog::kgpkubs(Pose initialPose, Pose finalPose, int &vl, int &vr)
{
    int clearance = CLEARANCE_PATH_PLANNER;
    static float prevVel = 0;
    Vector2D<int> initial(initialPose.x(), initialPose.y());
    Vector2D<int> final(finalPose.x(), finalPose.y());
    double curSlope = initialPose.theta();
    double finalSlope = finalPose.theta();
    double theta = normalizeAngle(Vector2D<int>::angle(final, initial));
    double phiStar = finalSlope;
    double phi = curSlope;
    //directionalAngle = (cos(atan2(final.y - initial.y, final.x - initial.x) - curSlope) >= 0) ? curSlope : normalizeAngle(curSlope - PI);
    int dist = Vector2D<int>::dist(final, initial);  // Distance of next waypoint from the bot
    double alpha = normalizeAngle(theta - phiStar);
    double beta = (fabs(alpha) < fabs(atan2(clearance, dist))) ? (alpha) : SGN(alpha) * atan2(clearance, dist);
    double thetaD = normalizeAngle(theta + beta);
    float delta = normalizeAngle(thetaD - phi);
    double r = (sin(delta) * sin(delta)) * SGN(tan(delta));
    double t = (cos(delta) * cos(delta)) * SGN(cos(delta));
    if(!(t >= -1.0 && t <= 1.0 && r >= -1.0 && r <= 1.0)) {        
     printf("what? delta = %f, initial = (%lf, %lf, %lf)\n", delta, initialPose.x(), initialPose.y(), initialPose.theta());
     assert(0);
    }
    float fTheta = asin(sqrt(fabs(r)));
    fTheta = 1 - fTheta/(PI/2);
    fTheta = pow(fTheta,2.2) ;
    float fDistance = (dist > BOT_POINT_THRESH*3) ? 1 : dist / ((float) BOT_POINT_THRESH *3);
    float fTot = fDistance * fTheta;
    fTot = 0.2 + fTot*(1-0.2);
    float profileFactor = MAX_BOT_SPEED * fTot;
    if(fabs(r)<0.11)
      profileFactor*=2;
    {
      if(fabs(profileFactor-prevVel)>MAX_BOT_LINEAR_VEL_CHANGE)
      {
        if(profileFactor>prevVel)
            profileFactor=prevVel+MAX_BOT_LINEAR_VEL_CHANGE;
        else
            profileFactor=prevVel-MAX_BOT_LINEAR_VEL_CHANGE;
      }
      prevVel=profileFactor;
    }
    if(profileFactor>1.5*MAX_BOT_SPEED)
      profileFactor = 1.5*MAX_BOT_SPEED;
    else if(profileFactor <-1.5*MAX_BOT_SPEED)
      profileFactor = -1.5*MAX_BOT_SPEED;
    prevVel=profileFactor;
    r *= 0.5*profileFactor;
    t *= profileFactor;

    vl = t-r;
    vr = t+r;
}

void Dialog::CMU(Pose s, Pose e, int &vl, int &vr)
{
    int maxDis = 2*sqrt(HALF_FIELD_MAXX*HALF_FIELD_MAXX+HALF_FIELD_MAXY*HALF_FIELD_MAXY);
    Vector2D<int> initial(s.x(), s.y());
    Vector2D<int> final(e.x(), e.y());
    int distance = Vector2D<int>::dist(initial, final);
    double phi = s.theta();
    double theta = Vector2D<int>::angle(final, initial);
    double phiStar = e.theta();
    double alpha = normalizeAngle(theta-phiStar);
    double beta = atan2((maxDis/15.0), distance);
    double thetaD = theta + min(alpha, beta);
    thetaD = theta; // removing the phiStar part. wasnt able to get it to work right.
    double delta = normalizeAngle(thetaD - phi);
    double t = cos(delta)*cos(delta)*SGN(cos(delta));
    double r = sin(delta)*sin(delta)*SGN(sin(delta));
    vl = 100*(t-r);
    vr = 100*(t+r);
}

void Dialog::PController(Pose s, Pose e, int &vl, int &vr)
{
    Vector2D<int> initial(s.x(), s.y());
    Vector2D<int> final(e.x(), e.y());
    int distance = Vector2D<int>::dist(initial, final);
    double angleError = normalizeAngle(s.theta() - Vector2D<int>::angle(final, initial));
    double v = 0;
    int maxDis = 2*sqrt(HALF_FIELD_MAXX*HALF_FIELD_MAXX+HALF_FIELD_MAXY*HALF_FIELD_MAXY);
    if(angleError > PI/2) {
        v = 0;
    } else {
        if(distance > maxDis/2) {
            v = 100;
        } else {
            v = (distance/(double)maxDis)*90+10;
        }
    }
    double w = -1.5*angleError;
    v *= Pose::ticksToCmS;
    vl = v - Pose::d*w/2;
    vr = v + Pose::d*w/2;
    if(abs(vl) > 100 || abs(vr) > 100) {
        double max = abs(vl)>abs(vr)?abs(vl):abs(vr);
        vl = vl*100/max;
        vr = vr*100/max;
    }
}

void Dialog::PolarBased(Pose s, Pose e, int &vl, int &vr)
{
    Vector2D<int> initial(s.x()-e.x(), s.y()-e.y());
    double theta = normalizeAngle(s.theta() - e.theta());
    // rotate initial by -e.theta degrees;
    double newx = initial.x * cos(-e.theta()) - initial.y * sin(-e.theta());
    double newy = initial.x * sin(-e.theta()) + initial.y * cos(-e.theta());
    initial = Vector2D<int>(newx, newy);
    double rho = sqrt(initial.x*initial.x + initial.y*initial.y);
    double gamma = normalizeAngle(atan2(initial.y, initial.x) - theta + PI);
    double delta = normalizeAngle(gamma + theta);
    double k1 = 0.05, k2 = 4, k3 = 20;
    double v = k1*rho*cos(gamma);
    double w;
    if(gamma == 0) {
        w = k2*gamma+k1*cos(gamma)*(gamma+k3*delta);
    } else {
        w = k2*gamma+k1*sin(gamma)*cos(gamma)/gamma*(gamma + k3*delta);
    }
    v *= Pose::ticksToCmS;
    vl = v - Pose::d*w/2;
    vr = v + Pose::d*w/2;
    double timeMs = 0.255 * rho; // empirical
    double speed = timeMs<16*5?timeMs/16*(80/5):80;
    double max = fabs(vl)>fabs(vr)?fabs(vl):fabs(vr);
    if(max > 0) {
        vl = vl*speed/max;
        vr = vr*speed/max;
    }
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
        (this->*func)(poses[i], endPose, vl, vr);
        vls[i-1] = vl;
        vrs[i-1] = vr;
        if(dist(poses[i], endPose) < 40 && !endFlag) {
//            qDebug() << "Bot reaches end pose after " << i << "frames (dist = " << dist(poses[i], endPose) << ")";
            timeMs = i*16;
            endFlag = 1;
        }
        poses[i].update(vl, vr, timeLC);
    }
    return timeMs;
}


void Dialog::batchSimulation(Dialog::FType fun)
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
            x2 = y2 = theta2 = 0; // doing this for regression with gamma and delta
            double rho = -rand()%HALF_FIELD_MAXX; // keeping it left side.
            double angle = rand()/(double)RAND_MAX;
            angle = normalizeAngle(angle* 2 * PI);
            angle = 0; // always facing towards final angle.
            x1 = rho*cos(angle);
            y1 = rho*sin(angle);
            theta1 = rand()/(double)RAND_MAX;
            theta1 = normalizeAngle(theta1*PI);
            theta1 = PI/4;
//            theta1 = rand()%2?-theta1:theta1;
        }
        Pose start(x1, y1, theta1);
        Pose end(x2, y2, theta2);
        int timeMs = simulate(start, end, fun);
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
    timer->start(16);
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
    int timeMs = simulate(start, end, fun);
    qDebug() << "Bot reached at time tick = " << timeMs/16;
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
