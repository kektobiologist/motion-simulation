#include "simulation.hpp"
double Simulator::simulate(Pose startPose, Pose endPose, FType func, int start_vl, int start_vr, bool isBatch)
{
    poses[0] = startPose;
    //simulating behaviour for all ticks at once
    int endFlag = 0;
    double timeMs = std::numeric_limits<double>::max();
    ControllerWrapper dc(func, start_vl, start_vr, Constants::numPacketDelay);
    for(int i=1; i < NUMTICKS; i++)
    {
        poses[i] = poses[i-1];
        int vl, vr;
        // NOTE: setting final velocity hardcoded here.
        miscData[i] = dc.genControls(poses[i], endPose, vl, vr, FINAL_VEL);
        vls[i-1] = vl;
        vrs[i-1] = vr;
        poses[i].update(vl, vr, timeLC);
        VisionVelocity::calcBotVelocity(poses[i-1], poses[i], timeLCMs, vls_calc[i-1], vrs_calc[i-1]);
        if(dist(poses[i], endPose) < 40 && !endFlag) {
            timeMs = i*timeLCMs;
            endFlag = 1;
            if(isBatch)
                break;
        }
    }
    return timeMs;
}

double Simulator::simulate(Pose startPose, Trajectory traj, int start_vl, int start_vr, bool isBatch)
{
    Q_UNUSED(isBatch)
    poses[0] = startPose;
    //simulating behaviour for all ticks at once
    double timeMs = std::numeric_limits<double>::max();
    ControllerWrapper dc(traj, start_vl, start_vr, Constants::numPacketDelay);
    for(int i=1; i < NUMTICKS; i++)
    {
        poses[i] = poses[i-1];
        int vl, vr;
        // NOTE: setting final velocity hardcoded here.
        miscData[i] = dc.genControlsTrajSim(poses[i], vl, vr, i*timeLCMs/1000.0);
        vls[i-1] = vl;
        vrs[i-1] = vr;
        poses[i].update(vl, vr, timeLC);
        VisionVelocity::calcBotVelocity(poses[i-1], poses[i], timeLCMs, vls_calc[i-1], vrs_calc[i-1]);
    }
    return timeMs;
}

QString Simulator::batchSimulation(FType fun)
{

    vector<RegData> func; // (dist,theta) maps to timeMs
    QString retString;
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
        int start_vl = rand()%((int)MAX_BOT_SPEED+1) * (rand()%2?-1:1);
        int start_vr = rand()%((int)MAX_BOT_SPEED+1) * (rand()%2?-1:1);
        // just set start vel as max only...
        start_vl = start_vr = MAX_BOT_SPEED;

        double timeMs = simulate(start, end, fun, start_vl, start_vr, true);
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
            func.push_back(RegData(rho, gamma, delta, start_vl, start_vr, timeMs));
            char buf[1000];
            sprintf(buf, "%g %g %g %g", rho, fabs(gamma), fabs(delta), timeMs);
            retString.append(buf);
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
    return retString;
}


void Simulator::regression(vector<RegData> func)
{
    int n = func.size();
    gsl_matrix *X = gsl_matrix_calloc(n, 4);
    gsl_vector *Y = gsl_vector_alloc(n);
    gsl_vector *beta = gsl_vector_alloc(4);

    for (int i = 0; i < n; i++) {
        gsl_vector_set(Y, i, func[i].timeMs);
        double v_trans = (func[i].v_l + func[i].v_r)/2;
        double v_rot = (func[i].v_r - func[i].v_l)/Constants::d;
//        gsl_matrix_set(X, i, 0, 1);
        gsl_matrix_set(X, i, 0, pow(func[i].rho, 1));
        gsl_matrix_set(X, i, 1, pow(func[i].rho, 1/2.0));
        gsl_matrix_set(X, i, 2, pow(fabs(v_trans), 1));
        gsl_matrix_set(X, i, 3, pow(fabs(v_rot), 1));
//        gsl_matrix_set(X, i, 4, pow(fabs(func[i].gamma), 2));
//        gsl_matrix_set(X, i, 5, pow(fabs(func[i].delta), 2));
//        gsl_matrix_set(X, i, 6, pow(fabs(normalizeAngle(func[i].gamma - func[i].delta)), 2));
//        gsl_matrix_set(X, i, 1, func[i].gamma);
//        gsl_matrix_set(X, i, 1, func[i].delta);
    }

    double chisq;
    gsl_matrix *cov = gsl_matrix_alloc(4, 4);
    gsl_multifit_linear_workspace * wspc = gsl_multifit_linear_alloc(n, 4);
    gsl_multifit_linear(X, Y, beta, cov, &chisq, wspc);
    qDebug() << "Beta = " << gsl_vector_get(beta, 0)
             << ", " << gsl_vector_get(beta, 1)
             << ", " << gsl_vector_get(beta, 2)
             << ", " << gsl_vector_get(beta, 3)
//             << ", " << gsl_vector_get(beta, 4)
//             << ", " << gsl_vector_get(beta, 5)
//             << ", " << gsl_vector_get(beta, 6)
             <<  ", chisq = " << chisq;// << ", " << gsl_vector_get(beta, 2);

    gsl_matrix_free(X);
    gsl_matrix_free(cov);
    gsl_vector_free(Y);
    gsl_vector_free(beta);
    gsl_multifit_linear_free(wspc);
}

double Simulator::fitnessFunction(double k1, double k2, double k3)
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

