#ifndef __ROLLER_EYE_RECORDER_MGR_H__
#define __ROLLER_EYE_RECORDER_MGR_H__

#include <sqlite3.h>
#include "roller_eye/msgqueue.h"
#include "roller_eye/single_class.h"
#include "roller_eye/recorder_mp4.h"
#include "roller_eye/recorder_jpg.h"
#include "roller_eye/system_define.h"

#include"roller_eye/status_publisher.h"
#include <dynamic_reconfigure/client.h>
#include <roller_eye/ImageConfig.h>

#include"roller_eye/record_start.h"
#include"roller_eye/record_stop.h"
#include"roller_eye/record_get_files.h"
#include"roller_eye/record_get_file_path.h"
#include"roller_eye/record_get_file_num.h"
#include"roller_eye/record_delete_file.h"
#include"roller_eye/record_get_status.h"
#include "roller_eye/record_clean.h"


namespace roller_eye {

const int JPG_MIN_SIZE = 2000;
const int MP4_MIN_SIZE = 8000;
const int SNAPSHOT_MIN_INTERVAL = 3;
const int VIDEO_MIN_DURATION = 3;

const int REC_MSG_QUEUE_SIZE = 256;
const int FILE_ID_LEN = 16;

const std::string REC_FILES_DIR = string(ROLLER_EYE_DINAMIC_FILE_HOME_PATH) + "media_files/";
const std::string DB_NAME = REC_FILES_DIR + "MEDIA_FILES.DB";
const std::string DB_REC_TABLE_NAME = "file_list";

struct rec_request_t {
    record_start::Request reqStart;
    std::string file_id;
    std::string file_path;
    std::string thumb_id; //for video
    std::string thumb_path; //for video
    bool isUnlimited;  //when request.count == 0
    uint32_t cntUnlimited;
    uint32_t duration;   
};

enum {
    IDX_S = 0, //snapshot
    IDX_V = 1, //video recording
    TASK_NUM
};

enum{
    MSG_RECORD_START,
    MSG_RECORD_STOP,
    MSG_RECORD_GET_FILES,
    MSG_RECORD_GET_FILE_PATH,
    MSG_RECORD_GET_FILE_NUM,
    MSG_RECORD_DELETE_FILE,

    MSG_RECORD_STOP_LOOP
};


class RecDBHelper
{
public:
    RecDBHelper();
    ~RecDBHelper();
    void restoreDB(const char* files_dir);
    int openDB(const std::string& dbName, sqlite3** db);
    void createTable(const std::string& tableName);
    bool isTableExist(const std::string& tableName);
    int insertItem(const string& fileId, const string& path, int duration, int type, int64_t size, int64_t create_time);
    bool deleteItem(const string& fileId);
    string queryFilePath(const string& fileId);
    int queryTypeNum(int type);
    bool queryTypeList(int type, int start, int size, vector<record>& list);
    bool queryTypeList(int type, const string& startId, int size, vector<record>& list);
    void closeAndReload();

private:
    void load();
    void close();
    void printTable(const std::string& tableName);
    static int tableExistCB(void* pHandle,int iRet, char** szSrc, char** szDst);
    sqlite3 * m_db;
    sqlite3_stmt *m_stmtInsert;
    sqlite3_stmt *m_stmtQueryPath;
    sqlite3_stmt *m_stmtQueryTypeNum;
    sqlite3_stmt *m_stmtQueryAllNum;  //exclude thumb
    sqlite3_stmt *m_stmtQueryTypeList;
    sqlite3_stmt *m_stmtQueryAllList;  //exclude thumb
    sqlite3_stmt *m_stmtDelete;
};


class RecorderManager
{
public:
    void startMsgLoop();
    void stopMsgLoop();
    void msgLoop();
    void setComponent(ros::NodeHandle& nh, queue_proc proc);
    void startRecordMp4(rec_request_t* request, const std::string& file_id);
    void stopRecordMp4(bool fullStop=false);

    void startSnapshot(rec_request_t *request, const std::string& file_id);
    void stopSnapshot(bool fullStop=false);
    void printCurrTask();

    bool recordStartCB(record_start::Request& request, record_start::Response& response);
    bool recordStopCB(record_stop::Request& request, record_stop::Response& response);
    bool recordGetFilesCB(record_get_files::Request& request, record_get_files::Response& response);
    bool recordGetFilePathCB(record_get_file_path::Request& request, record_get_file_path::Response& response);
    bool recordGetFileNumCB(record_get_file_num::Request& request, record_get_file_num::Response& response);
    bool recordDeleteFileCB(record_delete_file::Request& request, record_delete_file::Response& response);
    bool recordGetStatusCB(record_get_status::Request& request, record_get_status::Response& response);
    bool recordCleanCB(record_clean::Request& request, record_clean::Response& response);

private:
    friend class SingleClass<RecorderManager>;
    ~RecorderManager();
    RecorderManager();
    RecorderManager(const RecorderManager&);
    RecorderManager& operator=(const RecorderManager&);

    void resetCurrTask(int idx);
    void pubCurrTaskStatus();
    void startDiskCheck();
    bool diskIsChecking();
    bool isDupReqStart(record_start::Request& request);
    void sTimerCallback(const ros::TimerEvent& event);
    void vTimerCallback(const ros::TimerEvent& event);
    inline int typeMap(int msgtype);
    void thumbCallback(roller_eye::frameConstPtr frame);
    void imageCfgCallback(const roller_eye::ImageConfig &config);

    static const char* REC_TAG;
    ros::NodeHandle m_nh;
    ros::Subscriber m_subH264;
    ros::Subscriber m_subAAC;
    ros::Subscriber m_subJPG;
    ros::Subscriber m_subThumb;
    ros::Timer m_recTimer[TASK_NUM];
    std::mutex m_mutex[TASK_NUM];

    boost::shared_ptr<RecorderJPG> m_recordjpg;
    boost::shared_ptr<RecorderMP4> m_recordmp4;

    std::atomic<bool> m_msgRunning;
	std::thread m_msgThread;
    QHandle m_queue;

    rec_request_t m_currTask[TASK_NUM];

    RecDBHelper m_dbHelper;
    std::shared_ptr<StatusPublisher> mStatusPub;
    dynamic_reconfigure::Client<roller_eye::ImageConfig> mCfgClient;
    roller_eye::ImageConfig mImgCfg;
    bool  mSchedRecord;
};


}

#endif // __ROLLER_EYE_RECORDER_MGR_H__