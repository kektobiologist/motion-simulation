#include "controllers.h"
#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include <utility>
#include <math.h>
#include <QtCore/QString>
#include <QtCore/QFile>
#include <QtCore/QDebug>
#include <QtCore/QTextStream>

using namespace std;
namespace Controllers {

MiscData kgpkubs(Pose initialPose, Pose finalPose, int &vl, int &vr, double prevSpeed, double prevOmega, double finalSpeed)
{
    Q_UNUSED(prevSpeed);
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
    return MiscData();
}

MiscData CMU(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double prevOmega, double finalSpeed)
{
    Q_UNUSED(prevSpeed);
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
    return MiscData();
}


QString outputFilename = "/home/robocup/Output_QT.txt";
QFile outputFile(outputFilename);


int va=-1;
MiscData DynamicWindow(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double prevOmega, double finalSpeed)
{
//    qDebug()<<"prevSpeed = "<<prevSpeed<<" prevOmega = "<<prevOmega;
//    qDebug()<<"Inside function Dynamic Window";
//    sprintf(buf, "in function Dynamic Window");


    QTextStream outStream(&outputFile);
    if(va==-1)
    {
         outputFile.open(QIODevice::WriteOnly);
        outStream<<"\n\n\n\n \t\t\t\t The Print of QT Code\n";
        outStream<<"old_x \t old_y \t prevSpeed \t prevOmega \t newSpeed \t newOmega \t new_x \t new_y \n";
        va=0;
    }
    va=0;
     outputFile.open(QIODevice::Append);
     outStream<< s.x() << '\t'<< s.y() <<'\t'<< prevSpeed <<'\t'<< prevOmega<< '\t';
    const int del_v_max = 12; //ticks
    const float step = 1; //ticks
    const float max_vel = 100; //ticks
    const float a_r_max = 380; //cm/s^2
//    const float PI= 3.14159;
    const float t=0.016;
    const float k=5;
    int count=0;
    double arr[100000][3];
    //float prevSpeed, prevOmega,alpha,acc,theta,x,y,reqtheta,dtheta;

    //int i,j;                                                           //i=vr,,j=vl

    for(float del_vr=-del_v_max;del_vr<=del_v_max;del_vr+=step)
        {
            for(float del_vl=-del_v_max;del_vl<=del_v_max;del_vl+=step)
            {

                float newSpeed= prevSpeed + Constants::ticksToCmS*(del_vr+del_vl)/2; // cm/s
                float newOmega= prevOmega + Constants::ticksToCmS*(del_vr-del_vl)/Constants::d;  // cm/s                //D Is the constant breadth of the bot
//                qDebug()<<"Trying newSpeed = "<<newSpeed<<" newOmega = "<<newOmega;
                if(abs(newSpeed/Constants::ticksToCmS) >max_vel || abs(newOmega/Constants::ticksToCmS) >(2*max_vel)/Constants::d)
                    continue;
                if((newSpeed*newOmega)>=a_r_max*sqrt(1-pow((del_vr + del_vl)/(2*del_v_max),2)))
                    continue;                   // constraint from the equation of ellipse.
                //if(count>400) break;                                 //we would take only a fixed number of points under consideration
//                qDebug()<<"Trying newSpeed = "<<newSpeed<<" newOmega = "<<newOmega;
                // float alpha=(newOmega-prevOmega)/t; // rad/cm^2
                float theta= s.theta() + (newOmega*t); //rad
                //float acc_x=(newSpeed*cos(theta)-prevSpeed*cos(s.theta()))/t; //cm/s^2
                //float acc_y=(newSpeed*sin(theta)-prevSpeed*sin(s.theta()))/t; //cm/s^2
                theta = normalizeAngle(theta);
                //float x= s.x() + (prevSpeed*cos(theta)*t) + (0.5*acc_x*t*t);
                //float y= s.y() + (prevSpeed*sin(theta)*t) + (0.5*acc_y*t*t);

                 float x= s.x() + (newSpeed*cos(theta)*t);
                 float y= s.y() + (newSpeed*sin(theta)*t);
//                qDebug()<<"x"<<s.x()<<" --> "<<x<<" y "<<s.y()<<" --> "<<y;

                float reqtheta=atan2((e.y()-y),(e.x()-x));

                float dtheta=firaNormalizeAngle(theta-reqtheta);

                arr[count][0]= sqrt(pow((x - e.x()),2) + pow((y - e.y()),2)) + k*pow((dtheta),2) ;         // our objective function
                arr[count][1]= newSpeed;
                arr[count][2]= newOmega;
                count++;
//                qDebug()<<"obj = "<<arr[count][0];
            }
        }
    float min= arr[0][0];
    float best_v=arr[0][1];
    float best_w=arr[0][2];
    for(int i=0;i<count;i++)
    {
        if(arr[i][0]<min)
        {
            min=arr[i][0];
            best_v= arr[i][1];
            best_w = arr[i][2];
        }
    }
    float theta= s.theta() + (best_w*t); //rad
    theta = normalizeAngle(theta);
    float x= s.x() + (best_v*cos(theta)*t);
    float y= s.y() + (best_v*sin(theta)*t);

    outStream<<best_v<< '\t'<< best_w <<'\t'<< x <<'\t'<< y<<endl;

    vr=(best_v)+(Constants::d *best_w)/2 ;                    // update velocity
    vl=(2*best_v) - vr;                       // update omega
    vr/=Constants::ticksToCmS;
    vl/=Constants::ticksToCmS;

    return MiscData();
}

//MiscData DynamicWindow(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double prevOmega, double finalSpeed)
//{
//    const int del_v = 10;
//    const float step = 1;
//    int count=0;
//    double arr[1000][3];
//    //float oldvel, oldome,alpha,acc,theta,x,y,reqtheta,dtheta;
//    float t=0.016;
//    //int i,j;                                                           //i=vr,,j=vl
//    int k=5;
//    for(float del_vr=-del_v;i<=del_v;i+=step)
//        {
//            for(float j=-del_v;j<=del_v;j+=step)
//            {
//                //oldvel=prevSpeed;
//                //oldome=prevOmega;
//                newSpeed= prevSpeed + Constants::ticksToCmS*(i+j)/2;
//                newOmega= prevOmega + Constants::ticksToCmS*(i-j)/Constants::d;                  //D Is the constant breadth of the bot
//                if(abs(prevSpeed/Constants::ticksToCmS) >100 || abs(prevOmega/Constants::ticksToCmS) >200/Constants::d) continue;
//                if((prevSpeed*prevOmega)>=380*sqrt(0.99)) continue;                   // constraint from the equation of ellipse.
//                //if(count>400) break;                                 //we would take only a fixed number of points under consideration
//                count++;
//                alpha=(prevOmega-oldome)/t;
//                acc=(prevSpeed-oldvel)/t;
//                theta= s.theta() + (oldome*t) + (0.5*alpha*t*t);
//                if(theta>2*3.14){theta=theta - 2*3.14*static_cast<int>(theta/(2*3.14));}
//                x= s.x() + (oldvel*cos(theta)*t) + (0.5*acc*cos(theta)*t*t);
//                y= s.y() + (oldvel*sin(theta)*t) + (0.5*acc*sin(theta)*t*t);
//                reqtheta=tanh((e.y()-y)/(e.x()-x));
//                dtheta=abs(theta-reqtheta);
//                if(dtheta>2*3.14){dtheta=dtheta-2*3.14*static_cast<int>(theta/(2*3.14));}
//                if(dtheta>3.14)
//                      {dtheta=3.14*2-dtheta;}
//        arr[count][0]= pow((x - e.x()),2) + pow((y - e.y()),2) + k*pow((dtheta),2) ;         // our objective function
//        arr[count][1]= prevSpeed;
//        arr[count][2]= prevOmega;
//            }
//        }
//    float min= arr[0][0];
//    for(i=1;i<count;i++)
//    {
//        if(arr[i][0]<min) { min=arr[i][0]; j=i; }
//    }
//    vr=(arr[j][1])+(Constants::d * arr[j][2])/2 ;                    // update velocity
//    vl=(2*arr[j][1]) - vr;                       // update omega
//    vr/=Constants::ticksToCmS;
//    vl/=Constants::ticksToCmS;

//    return MiscData();
//}

MiscData PController(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double prevOmega, double finalSpeed)
{
    Q_UNUSED(prevSpeed);
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
            v = MAX_BOT_SPEED;
        } else {
            v = (distance/(double)maxDis)*90+10;
        }
    }
    double w = -1.5*angleError;
    v *= Constants::ticksToCmS;
    vl = v - Constants::d*w/2;
    vr = v + Constants::d*w/2;
    if(abs(vl) > MAX_BOT_SPEED || abs(vr) > MAX_BOT_SPEED) {
        double max = abs(vl)>abs(vr)?abs(vl):abs(vr);
        vl = vl*MAX_BOT_SPEED/max;
        vr = vr*MAX_BOT_SPEED/max;
    }
    return MiscData();
}

/*  Aicardi M, Casalino G, Bicchi A, Balestrino A 1995 Closed loop steering of
unicycle-like vehicles via Lyapunov techniques. IEEE Robotics & Automation
Magazine 2(1):27–35
*/
MiscData PolarBased(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double prevOmega, double finalSpeed)
{
    // NOTE: its preferable to call x(), y(), and theta() of each object exactly once since they may return different
    // values on each call (when simulating, gaussian errors non-zero).
    Vector2D<int> initial(s.x()-e.x(), s.y()-e.y());
    double etheta = e.theta();
    double theta = normalizeAngle(s.theta() - etheta);
    // rotate initial by -e.theta degrees;
    double newx = initial.x * cos(-etheta) - initial.y * sin(-etheta);
    double newy = initial.x * sin(-etheta) + initial.y * cos(-etheta);
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
    // velocity profiling based on curvature
    double k = w/v; // k = curvature
    // scale curvature by 50.
    k *= 20;
    double lambda = 2;
    double beta = 0.7;
    double v_curve = MAX_BOT_SPEED/(1+beta*pow(fabs(k),lambda));
    if (v_curve < MIN_BOT_SPEED)
        v_curve = MIN_BOT_SPEED;
    v *= Constants::ticksToCmS;
    vl = v - Constants::d*w/2;
    vr = v + Constants::d*w/2;
    double timeMs = 0.250*rho + 14.0 * sqrt(rho) + 100.0 * fabs(gamma);
    double speed = timeMs/timeLCMs<(prevSpeed/MAX_BOT_LINEAR_VEL_CHANGE)?prevSpeed-MAX_BOT_LINEAR_VEL_CHANGE:prevSpeed+MAX_BOT_LINEAR_VEL_CHANGE;
    // use vcurve as the velocity
    // NOTE: adding vcurve and finalVel code
    // critical condition: if bot close to final point, v_curve = MAX_BOT_SPEED
    if (rho < BOT_POINT_THRESH && finalSpeed > MIN_BOT_SPEED) {
        v_curve = MAX_BOT_SPEED;
        vl = vr; // ? or vl = vr?
    }
    double rangeMin = max(prevSpeed - MAX_BOT_LINEAR_VEL_CHANGE, 0.0);
    double rangeMax = min(prevSpeed + MAX_BOT_LINEAR_VEL_CHANGE, v_curve);
    if (v_curve < prevSpeed - MAX_BOT_LINEAR_VEL_CHANGE) {
        rangeMin = rangeMax = v_curve;
    }
    // only consider finalSpeed when d < BOT_FINALVEL_THRESH
    if (rho <= BOT_FINALVEL_THRESH)
        speed = finalSpeed < rangeMax? max(rangeMin, finalSpeed) : min(rangeMax, finalSpeed);
    else
        speed = rangeMax;
    // critical condition : finalVel = 0, but bot not close to final pose
    if (rho > BOT_POINT_THRESH && speed < MIN_BOT_SPEED)
        speed = MIN_BOT_SPEED;

    // end
    if(speed > MAX_BOT_SPEED)
        speed = MAX_BOT_SPEED;
    else if (speed < 0)
        speed = 0;
    double max = fabs(vl)>fabs(vr)?fabs(vl):fabs(vr);
    if(max > 0) {
        vl = vl*speed/max;
        vr = vr*speed/max;
    }
    return MiscData(k, v_curve, finalSpeed, rangeMin, rangeMax);
}
MiscData PolarBidirectional(Pose s, Pose e, int &vl, int &vr, double prevSpeed, double prevOmega, double finalSpeed)
{
    MiscData m;
    static bool wasInverted = false; // keeps track of whether in the previous call, the bot was inverted or not.
    m = PolarBased(s, e, vl, vr, prevSpeed, finalSpeed);
    double v = (vl+vr)/2.0;
    wasInverted = false;
    if(v < 0 || (v == 0 && wasInverted) ) {
        s.setTheta(normalizeAngle(s.theta()+PI));
        m = PolarBased(s, e, vl, vr, prevSpeed, finalSpeed);
        swap(vl, vr);
        vl = -vl;
        vr = -vr;
        wasInverted = true;
    }
    return m;
}

void PolarBasedGA(Pose s, Pose e, int &vl, int &vr, double k1, double k2, double k3) // this function is old now, do not use.
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
    double v = k1*rho*cos(gamma);
    double w;
    if(gamma == 0) {
        w = k2*gamma+k1*cos(gamma)*(gamma+k3*delta);
    } else {
        w = k2*gamma+k1*sin(gamma)*cos(gamma)/gamma*(gamma + k3*delta);
    }
    v *= Constants::ticksToCmS;
    vl = v - Constants::d*w/2;
    vr = v + Constants::d*w/2;
    double timeMs = 23 * sqrt(rho); // empirical
    double speed = timeMs<timeLCMs*5?timeMs/timeLCMs*(80/5):80;
    double max = fabs(vl)>fabs(vr)?fabs(vl):fabs(vr);
    if(max > 0) {
        vl = vl*speed/max;
        vr = vr*speed/max;
    }
}

}
