// Utility function for logging
#ifndef LOGGING_HPP
#define LOGGING_HPP
#include "logging.pb.h"
#include <vector>

using namespace std;

namespace Logging {
// sysData = list of data stored by server
// recvData = list of data received from bot
// this function merges these lists into a list of LoggingData, making sure only corresponding packets from sys, recv
// are merged together. hence vector<LoggingData> .size() == recvData.size() <= sysData.size()
// since some packets might be lost by bot.
vector<LoggingData> mergeSysRecvLists(const vector<SystemData> &sysData, const vector<ReceivedData> &recvData);
}

#endif // LOGGING_HPP
