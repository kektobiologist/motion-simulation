// Utility function for logging
#ifndef LOGGING_HPP
#define LOGGING_HPP
#include "logging.pb.h"
#include <vector>
#include "beliefstate.h"

using namespace std;

namespace Logging {
// sysData = list of data stored by server
// recvData = list of data received from bot
// this function merges these lists into a list of LoggingData, making sure only corresponding packets from sys, recv
// are merged together. some sysData packets might be missing recvData due to packet loss / small memory of bot, which stores
// only ~200 packets of recvData at a time.
// TODO: very naive merging, make it better
Log mergeSysRecvLists(const vector<SystemData> &sysData, const vector<ReceivedData> &recvData);

// make system data packet from given information.
SystemData populateSystemData(int ts, int vl_sent, int vr_sent, const BeliefState &bs, int botid);

// make recv data packet from given information.
ReceivedData populateReceivedData(int botid, int ts, int vl_target, int vr_target, int vl, int vr);
}

#endif // LOGGING_HPP
