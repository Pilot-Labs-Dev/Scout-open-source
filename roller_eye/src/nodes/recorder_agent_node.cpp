#include "ros/ros.h"
#include "roller_eye/recorder_mgr.h"
#include"roller_eye/system_define.h"

using namespace roller_eye;
using namespace std;

static RecorderManager* recordMgr;

static void msgProcess(pqueue_msg *msg)
{
    //std::cout << "msgProcess: " << msg->msg << std::endl;

    switch (msg->msg) {
        case MSG_RECORD_START:
        {
            rec_request_t* request = static_cast<rec_request_t*>(msg->parc);
            ROS_INFO("record_start. type: %d, duration: %d\n", request->reqStart.type, request->reqStart.duration);
            if (request->reqStart.type == record::RECORD_TYPE_RECORD) {
                recordMgr->startRecordMp4(request, "");
            } else if (request->reqStart.type == record::RECORD_TYPE_SNAPSHOT) {
                recordMgr->startSnapshot(request, "");
            } else {
                ROS_ERROR("Not defined type: %d\n", request->reqStart.type);
            }
            delete request;
            break;
        }
        case MSG_RECORD_STOP:
        {
            // record_stop::Request* request = static_cast<record_stop::Request*>(msg->parc);
            int reqtype = msg->pars;
            ROS_INFO("record_stop. type: %d \n", reqtype);
            if (reqtype == record::RECORD_TYPE_RECORD) {
                recordMgr->stopRecordMp4(true);
            } else if (reqtype == record::RECORD_TYPE_SNAPSHOT) {
                recordMgr->stopSnapshot(true);
            } else {
                ROS_INFO("Not defined type: %d\n", reqtype);
            }
            break;
        }
        case MSG_RECORD_GET_FILES:
            break;
        case MSG_RECORD_GET_FILE_PATH:
            break;
        case MSG_RECORD_GET_FILE_NUM:
            break;
        case MSG_RECORD_DELETE_FILE:
            break;
        case MSG_RECORD_STOP_LOOP:
            recordMgr->stopMsgLoop();
            break;
        default:
            ROS_INFO("not define msg: %d", msg->msg);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RecorderAgentNode");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,RECORD_NODE_DEBUG_LEVEL);
    ros::NodeHandle nh("~");

    recordMgr = SingleClass<RecorderManager>::getInstance();
    recordMgr->setComponent(nh, msgProcess);

    ros::ServiceServer recordStartSrv = nh.advertiseService("record_start", &RecorderManager::recordStartCB, recordMgr);
    ros::ServiceServer recordStopSrv = nh.advertiseService("record_stop", &RecorderManager::recordStopCB, recordMgr);
    ros::ServiceServer recordGetFilesSrv = nh.advertiseService("record_get_files", &RecorderManager::recordGetFilesCB, recordMgr);
    ros::ServiceServer recordGetFilePathSrv = nh.advertiseService("record_get_file_path", &RecorderManager::recordGetFilePathCB, recordMgr);
    ros::ServiceServer recordGetFileNumSrv = nh.advertiseService("record_get_file_num", &RecorderManager::recordGetFileNumCB, recordMgr);
    ros::ServiceServer recordDeleteFileSrv = nh.advertiseService("record_delete_file", &RecorderManager::recordDeleteFileCB, recordMgr);
    ros::ServiceServer recordGetStatusSrv = nh.advertiseService("record_get_status", &RecorderManager::recordGetStatusCB, recordMgr);
    ros::ServiceServer recordCleanSrv = nh.advertiseService("record_clean", &RecorderManager::recordCleanCB, recordMgr);

    recordMgr->startMsgLoop();
#if 0
    // for debug
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        recordMgr->printCurrTask();
        loop_rate.sleep();
        ros::spinOnce();
    }
#else
    ros::spin();
#endif
    recordStartSrv.shutdown();
    recordStopSrv.shutdown();
    recordGetFilesSrv.shutdown();
    recordGetFilePathSrv.shutdown();
    recordGetFileNumSrv.shutdown();
    recordDeleteFileSrv.shutdown();
    recordGetStatusSrv.shutdown();
    return 0;
}

