#include "logging.hpp"
#include "logging.pb.h"
#include <QDebug>
#include <algorithm>
#include <sys/time.h>

namespace Logging {

vector<LoggingData> mergeSysRecvLists(const vector<SystemData> &sysData, const vector<ReceivedData> &recvData) {
    if (recvData.size() > sysData.size()) {
        qDebug() << "SystemData < ReceivedData. Aborting.";
        return vector<LoggingData>();
    }
    int idx = sysData.size() - 1;  // corresponding idx in the sysData arr.
    vector<LoggingData> loggingData;
    for (int i = recvData.size() - 1; i >= 0; i--) {
        while (idx >= 0 && sysData[idx].ts() != recvData[i].ts()) {
            LoggingData data;
            *data.mutable_sysdata() = sysData[idx--];
            loggingData.push_back(data);
        }
        if (idx < 0) {
            qDebug() << "Could not find corresponding sysData for recvData" << i << ", ts = " << recvData[i].ts() << ". Stopping here.";
            break;
        }
        LoggingData data;
        *data.mutable_sysdata() = sysData[idx];
        *data.mutable_recvdata() = recvData[i];
        loggingData.push_back(data);
    }
    std::reverse(loggingData.begin(), loggingData.end());
    return loggingData;
}

SystemData populateSystemData(int ts, int vl_sent, int vr_sent, const BeliefState &bs, int botid) {
    struct timeval nowTime;
    gettimeofday(&nowTime, NULL);
    float elapsedMs = (nowTime.tv_sec*1000.0+nowTime.tv_usec/1000.0);
    Logging::SystemData data;
    data.set_ts(ts);
    data.set_timems(elapsedMs);
    {
        Logging::Velocities sent, vision;
        sent.set_vl(vl_sent);
        sent.set_vr(vr_sent);
        vision.set_vl(bs.homeVl[botid]);
        vision.set_vr(bs.homeVr[botid]);
        *data.mutable_sent() = sent;
        *data.mutable_vision() = vision;
    }
    {
        Logging::RobotPose pose;
        pose.set_x(bs.homeX[botid]);
        pose.set_y(bs.homeY[botid]);
        pose.set_theta(bs.homeTheta[botid]);
        *data.mutable_pose() = pose;
    }
    return data;
}

ReceivedData populateReceivedData(int botid, int ts, int vl_target, int vr_target, int vl, int vr) {
    Logging::ReceivedData data;
    data.set_id(botid);
    data.set_ts(ts);
    {
        Logging::Velocities target, measured;
        target.set_vl(vl_target);
        target.set_vr(vr_target);
        *data.mutable_target() = target;
        measured.set_vl(vl);
        measured.set_vr(vr);
        *data.mutable_measured() = measured;
    }
    return data;
}
}
