#include "logging.hpp"
#include "logging.pb.h"
#include <QDebug>
#include <algorithm>

namespace Logging {

vector<LoggingData> mergeSysRecvLists(const vector<SystemData> &sysData, const vector<ReceivedData> &recvData) {
    if (recvData.size() > sysData.size()) {
        qDebug() << "SystemData < ReceivedData. Aborting.";
        return vector<LoggingData>();
    }
    int idx = recvData.size() - 1;  // corresponding idx in the sysData arr.
    vector<LoggingData> loggingData;
    for (int i = recvData.size() - 1; i >= 0; i--) {
        while (idx >= 0 && sysData[idx].ts() != recvData[i].ts())
            idx--;
        if (idx < 0) {
            qDebug() << "Could not find corresponding sysData for recvData" << i << ". Stopping here.";
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

}
