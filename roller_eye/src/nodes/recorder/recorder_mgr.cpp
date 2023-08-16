#include "ros/ros.h"
#include "plog_plus.h"
#include "plterrno.h"
#include "plt_tools.h"
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include"plt_assert.h"
#include "roller_eye/recorder_mgr.h"
#include "roller_eye/system_define.h"


namespace roller_eye {

const char* RecorderManager::REC_TAG = "rec_mgr";

RecorderManager::RecorderManager(): m_nh(), m_recordmp4(nullptr), m_msgRunning(false), m_queue(nullptr),
mCfgClient(PARAM_VIDEO_PATH, boost::bind(&RecorderManager::imageCfgCallback, this, _1))
{
    mSchedRecord = false;

    PLOG_DEBUG(REC_TAG, "RecorderManager");
    if (0 != access(REC_FILES_DIR.c_str(), F_OK)) {
        if (0 != mkdir(REC_FILES_DIR.c_str(), 0755)) {
            perror("mkdir error");
            PLOG_ERROR(REC_TAG, "RecorderManager mkdir error: %s", REC_FILES_DIR.c_str());
        }
    }

    resetCurrTask(IDX_S);
    resetCurrTask(IDX_V);
    m_recTimer[IDX_S] = m_nh.createTimer(ros::Duration(60), &RecorderManager::sTimerCallback, this, false, false);
    m_recTimer[IDX_V] = m_nh.createTimer(ros::Duration(60), &RecorderManager::vTimerCallback, this, false, false);
    startDiskCheck();
}

RecorderManager::~RecorderManager()
{
    resetCurrTask(IDX_S);
    resetCurrTask(IDX_V);
    m_recTimer[IDX_S].stop();
    m_recTimer[IDX_V].stop();
    pqueue_destroy(m_queue);
}

void RecorderManager::imageCfgCallback(const roller_eye::ImageConfig& config)
{
      if (config.image_width != mImgCfg.image_width ||
            config.image_height != mImgCfg.image_height ||
            config.image_fmt != mImgCfg.image_fmt ||
            config.image_fps_den != mImgCfg.image_fps_den ||
            config.image_fps_num != mImgCfg.image_fps_num ||
            config.h264Quality != mImgCfg.h264Quality ||
            config.cameraLight != mImgCfg.cameraLight){
        stopRecordMp4(true);
      }
    mImgCfg = config;
}

void RecorderManager::setComponent(ros::NodeHandle& nh, queue_proc proc)
{
    m_queue = pqueue_create(REC_MSG_QUEUE_SIZE, proc);
    mStatusPub = std::make_shared<StatusPublisher>(nh);
    pubCurrTaskStatus();
}

void RecorderManager::startDiskCheck()
{
    const string cmdstr = "/usr/local/bin/check_rec_disk.sh " + REC_FILES_DIR + " " + m_nh.getNamespace();
    std::thread checkLoop([=]() {
        while (true) {
            if (0 != system(cmdstr.c_str())) {
                PLOG_ERROR(REC_TAG, "Check rec files failed!\n");
            }
            std::this_thread::sleep_for(chrono::seconds(60*10));
        }
    });
    checkLoop.detach();
}

bool RecorderManager::diskIsChecking()
{
    char buf[128];
    FILE *fp = nullptr;
    if ((fp = popen("ps -elf | grep 'check_rec_disk.sh' | grep -cv 'grep'", "r")) == NULL) {
        perror("popen");
        return false;
    }
    if (fgets(buf, sizeof(buf), fp) == NULL) {
        return false;
    }
    pclose(fp);
    return atoi(buf) > 0;
}

void RecorderManager::startMsgLoop()
{
	m_msgRunning = true;
	m_msgThread = std::thread(&RecorderManager::msgLoop, this);
}

void RecorderManager::stopMsgLoop()
{
	if(!m_msgRunning) {
		PLOG_WARN(REC_TAG, "msgLoop already stopped");
        return;
	}

	m_msgRunning = false;
    pqueue_msg msg;
    msg.msg = MSG_RECORD_STOP_LOOP;
    pqueue_push(m_queue, msg, 1);
	m_msgThread.join();
}

void RecorderManager::msgLoop()
{
    PLOG_INFO(REC_TAG, "RecorderManager::msgLoop");
    int ret;
	while (m_msgRunning) {
        ret = pqueue_pop_timeout(m_queue, -1);
        if (ret != 0) {
            PLOG_DEBUG(REC_TAG, "get msg failed: %s\n", pstr_lasterr());
            //usleep(100*1000);
            usleep(20*1000);
        }
	}
}

static string curr_time_str()
{
    char szBuf[256] = {0};
    time_t timer = time(NULL);
    strftime(szBuf, sizeof(szBuf), "%Y-%m-%d-%H-%M-%S", localtime(&timer));
    return string(szBuf);
}

static char *rand_str(char *str, const int len)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    srand(ts.tv_sec*1000 + ts.tv_nsec/1000);
    int i;
    for (i=0; i<len; ++i) {
        switch ((rand()%3)) {
        case 1:
            str[i]='A'+rand()%26;
        break;
        case 2:
            str[i]='a'+rand()%26;
        break;
        default:
            str[i]='0'+rand()%10;
        }
    }
    str[i] = '\0';
    return str;
}


void RecorderManager::sTimerCallback(const ros::TimerEvent& event)
{
    //stopSnapshot();
    if (!m_currTask[IDX_S].isUnlimited) {
        if (static_cast<int>(--m_currTask[IDX_S].reqStart.count) <= 0) {
            bool bHaveData = m_recordjpg->haveData();
            bool bTimeout = static_cast<int>(m_currTask[IDX_S].reqStart.count) < -300;
            if (bHaveData || bTimeout){
                stopSnapshot();
                PLOG_DEBUG(REC_TAG, "snapshot timer finished.\n");
                m_subJPG.shutdown();
                m_recTimer[IDX_S].stop();
                resetCurrTask(IDX_S);
                pubCurrTaskStatus();
            }
            return;
        }
        stopSnapshot();
        PLOG_DEBUG(REC_TAG, "remaining stimer count: %d", m_currTask[IDX_S].reqStart.count);
    } else {
        stopSnapshot();
        m_currTask[IDX_S].cntUnlimited++;
        PLOG_DEBUG(REC_TAG, "unlimited shooting, count: %d", m_currTask[IDX_S].cntUnlimited);
    }

    if (!m_recTimer[IDX_S].hasStarted()) {
        PLOG_DEBUG(REC_TAG, "stimer is stopped.\n");
        return;
    }

    //get new file id.
    char idStr[FILE_ID_LEN + 4];
    rand_str(idStr, FILE_ID_LEN);
    startSnapshot(nullptr, std::string(idStr));
}

void RecorderManager::startSnapshot(rec_request_t *request, const std::string& file_id)
{
    // from msg queue

    int cnt = 0;
    while(cnt++ < 50){
        if (m_currTask[IDX_S].reqStart.type != 0){
            usleep(20*1000);
        }else{
            break;
        }
    }

    if (request != nullptr) {
        stopSnapshot();
        m_currTask[IDX_S].file_id = request->file_id;
        m_currTask[IDX_S].reqStart = request->reqStart;
        m_currTask[IDX_S].isUnlimited = request->reqStart.count == 0;
        m_currTask[IDX_S].cntUnlimited = 0;

        if (request->reqStart.count == 1)
            m_recTimer[IDX_S].setPeriod(ros::Duration(0,10*1000*1000));
        else
            m_recTimer[IDX_S].setPeriod(ros::Duration(request->reqStart.duration));
    }

    // from timer callback
    if (!file_id.empty()) {
        m_currTask[IDX_S].file_id = file_id;
    }
    PLOG_INFO(REC_TAG, "RecorderManager::startSnapshot. file_id: %s", m_currTask[IDX_S].file_id.c_str());

    string fileName(REC_FILES_DIR);
    fileName.append(curr_time_str()).append("_" + m_currTask[IDX_S].file_id)
            .append("-" + to_string(m_currTask[IDX_S].reqStart.type)).append(".jpg");
    m_currTask[IDX_S].file_path = fileName;

    m_recordjpg = boost::make_shared<RecorderJPG>(fileName);

    m_subJPG = m_nh.subscribe("CoreNode/jpg", 100, &RecorderJPG::jpgCallback, m_recordjpg);
    if (!m_subJPG) {
        PLOG_ERROR(REC_TAG, "sub jpg failed.");
    }

    m_recordjpg->startLoop();
    m_recTimer[IDX_S].start();
    pubCurrTaskStatus();
}

void RecorderManager::stopSnapshot(bool fullStop)
{
    std::lock_guard<std::mutex> locker(m_mutex[IDX_S]);
    PLOG_INFO(REC_TAG,"RecorderManager::stopSnapshot \n");

	if (m_recordjpg) {
        m_recordjpg->setTrigger(true);
        m_recordjpg->stopLoop();
    }
    const std::string &fileID = m_currTask[IDX_S].file_id;
    const std::string &filePath = m_currTask[IDX_S].file_path;
    const record_start::Request *reqStart = &m_currTask[IDX_S].reqStart;
    if (!filePath.empty()) {
        struct stat statbuf;
        if (0 != stat(filePath.c_str(), &statbuf)) {
            perror("stat");
            PLOG_ERROR(REC_TAG, "stat jpg file error: %s", filePath.c_str());
            goto OUT;
        }
        if (statbuf.st_size < JPG_MIN_SIZE) {
            PLOG_WARN(REC_TAG, "Drop too small file: %s, size: %ld", filePath.c_str(), statbuf.st_size);
            if (remove(filePath.c_str())) perror("remove jpg");
            goto OUT;
        }
        PLOG_INFO(REC_TAG, "INSERT ITEM: %s, %s, %d, %d, %ld, %ld", fileID.c_str(),
            filePath.c_str(),
            reqStart->duration,
            reqStart->type,
            statbuf.st_size,
            statbuf.st_atim.tv_sec);
        int rc = m_dbHelper.insertItem(fileID,
            filePath,
            reqStart->duration,
            reqStart->type,
            statbuf.st_size,
            statbuf.st_atim.tv_sec);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG, "INSERT DB ERROR CODE: %d", rc);
            if (remove(filePath.c_str())) perror("remove jpg");
        }
    }
OUT:
    if (fullStop) {
        m_subJPG.shutdown();
        m_recTimer[IDX_S].stop();
        resetCurrTask(IDX_S);
        pubCurrTaskStatus();
    }
}

void RecorderManager::vTimerCallback(const ros::TimerEvent& event)
{
    ROS_DEBUG("vtimer. current_expected: %f", event.current_expected.toSec());
    ROS_DEBUG("vtimer. current_real: %f", event.current_real.toSec());

    stopRecordMp4();
    if (!m_currTask[IDX_V].isUnlimited) {
        if (--m_currTask[IDX_V].reqStart.count == 0) {
            PLOG_DEBUG(REC_TAG, "recording timer finished.\n");
            m_subH264.shutdown();
            m_subAAC.shutdown();
            m_recTimer[IDX_V].stop();
            resetCurrTask(IDX_V);
            pubCurrTaskStatus();
            return;
        }
        PLOG_DEBUG(REC_TAG, "remaining vtimer count: %d", m_currTask[IDX_V].reqStart.count);
    } else {
        m_currTask[IDX_V].cntUnlimited++;
        PLOG_DEBUG(REC_TAG, "unlimited recording, count: %d", m_currTask[IDX_V].cntUnlimited);
    }

    if (!m_recTimer[IDX_V].hasStarted()) {
        PLOG_DEBUG(REC_TAG, "vtimer is stopped.\n");
        return;
    }

    //get new file id.
    char idStr[FILE_ID_LEN + 4];
    rand_str(idStr, FILE_ID_LEN);
    startRecordMp4(nullptr, std::string(idStr));
}

void RecorderManager::startRecordMp4(rec_request_t *request, const std::string& file_id)
{
    // from msg queue
    if (request != nullptr) {
        stopRecordMp4();

        m_currTask[IDX_V].file_id = request->file_id;
        m_currTask[IDX_V].reqStart = request->reqStart;
        m_currTask[IDX_V].isUnlimited = request->reqStart.count == 0;
        m_currTask[IDX_V].cntUnlimited = 0;
        m_currTask[IDX_V].duration = request->reqStart.duration;
        m_recTimer[IDX_V].setPeriod(ros::Duration(request->reqStart.duration));
    }

    // from timer callback
    if (!file_id.empty()) {
        m_currTask[IDX_V].file_id = file_id;
    }
    PLOG_INFO(REC_TAG, "RecorderManager::startRecordMp4. file_id: %s", m_currTask[IDX_V].file_id.c_str());
    string currTimeStr = curr_time_str();

    // prepare thumb handler -----start
    char idStr[FILE_ID_LEN + 4];
    rand_str(idStr, FILE_ID_LEN);
    m_currTask[IDX_V].thumb_id = string(idStr);

    string thumbName(REC_FILES_DIR);
    thumbName.append(currTimeStr + "_" + m_currTask[IDX_V].thumb_id)
            .append("-" + to_string(record::RECORD_TYPE_THUMB)).append(".jpg");
    m_currTask[IDX_V].thumb_path = thumbName;

    m_subThumb = m_nh.subscribe("CoreNode/jpg", 100, &RecorderManager::thumbCallback, this);
    if (!m_subThumb) {
        PLOG_ERROR(REC_TAG, "sub jpg failed.");
    }
    // prepare thumb handler -----end

    string fileName(REC_FILES_DIR);
    fileName.append(currTimeStr + "_" + m_currTask[IDX_V].file_id)
            .append("_" + m_currTask[IDX_V].thumb_id).append(".mp4");
    m_currTask[IDX_V].file_path = fileName;

    m_recordmp4 = boost::make_shared<RecorderMP4>(fileName, mImgCfg.image_width, mImgCfg.image_height, (float)mImgCfg.image_fps_den/mImgCfg.image_fps_num);

    m_subH264 = m_nh.subscribe("CoreNode/h264", 100, &RecorderMP4::h264Callback, m_recordmp4);
    if (!m_subH264) {
        PLOG_ERROR(REC_TAG, "sub h264 failed.");
    }
    m_subAAC = m_nh.subscribe("CoreNode/aac", 100, &RecorderMP4::aacCallback, m_recordmp4);
    if (!m_subAAC) {
        PLOG_ERROR(REC_TAG, "sub aac failed.");
    }
    m_recordmp4->startLoop();
    m_recTimer[IDX_V].start();
    pubCurrTaskStatus();
}

void RecorderManager::stopRecordMp4(bool fullStop)
{
    std::lock_guard<std::mutex> locker(m_mutex[IDX_V]);
    PLOG_INFO(REC_TAG, "RecorderManager::stopRecordMp4 \n");

    if (m_recordmp4) {
        m_recordmp4->stopLoop();
    }
    const std::string &fileID = m_currTask[IDX_V].file_id;
    const std::string &filePath = m_currTask[IDX_V].file_path;
    const record_start::Request *reqStart = &m_currTask[IDX_V].reqStart;
    const std::string &thumbID = m_currTask[IDX_V].thumb_id;
    const std::string &thumbPath = m_currTask[IDX_V].thumb_path;
    if (!filePath.empty()) {
        int rc = 0;
        struct stat statbuf;
        if (0 != stat(filePath.c_str(), &statbuf)) {
            perror("stat");
            PLOG_ERROR(REC_TAG, "stat mp4 file error: %s", filePath.c_str());
            goto OUT;
        }
        if (statbuf.st_size < MP4_MIN_SIZE) {
            PLOG_WARN(REC_TAG, "Drop too small file: %s, size: %ld", filePath.c_str(), statbuf.st_size);
            rc = remove(filePath.c_str());
            if (rc) perror("remove mp4");

            // delete thumb in fs&db
            rc = remove(thumbPath.c_str());
            if (rc) perror("remove mp4 thumb");
            if (false == m_dbHelper.deleteItem(thumbID)) {
                PLOG_ERROR(REC_TAG, "Delete db item failed: %s", thumbID.c_str());
            }
            goto OUT;
        }

        struct stat statThumb;
        if (0 != stat(thumbPath.c_str(), &statThumb)) {
            perror("stat");
            PLOG_ERROR(REC_TAG, "stat thumb file error: %s", thumbPath.c_str());
            goto OUT;
        }
        std::string newFilePath(filePath);
        newFilePath.insert(newFilePath.rfind(".mp4"), "-" + to_string(statThumb.st_size));
        rc = rename(filePath.c_str(), newFilePath.c_str());
        if (rc) perror("rename mp4");

        int dur = RecorderMP4::getFileDuration(newFilePath.c_str());
        PLOG_INFO(REC_TAG, "INSERT ITEM: %s, %s, %d, %d, %ld, %ld", fileID.c_str(),
            newFilePath.c_str(),
            dur,
            reqStart->type,
            statbuf.st_size,
            statbuf.st_atim.tv_sec);
        rc = m_dbHelper.insertItem(fileID,
            newFilePath,
            dur,
            reqStart->type,
            statbuf.st_size,
            statbuf.st_atim.tv_sec);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG, "INSERT DB ERROR CODE: %d", rc);
            rc = remove(newFilePath.c_str());
            if (rc) perror("remove mp4");
        }
    }
OUT:
    if (fullStop) {
        m_subH264.shutdown();
        m_subAAC.shutdown();
        m_subThumb.shutdown();
        m_recTimer[IDX_V].stop();
        resetCurrTask(IDX_V);
        pubCurrTaskStatus();
    }
}

void RecorderManager::thumbCallback(roller_eye::frameConstPtr frame)
{
	struct timespec tpTime;
    clock_gettime(CLOCK_MONOTONIC, &tpTime);
    uint64_t nowTime = tpTime.tv_sec*1000 + tpTime.tv_nsec/1000000;
    ROS_DEBUG("thumbCallback: %u, stamp: %ld, datasize: %lu, timediff: %ld", frame->seq, frame->stamp, frame->data.size(), nowTime - frame->stamp);

    FILE *fp_save = nullptr;
    if (frame->data.size() <= 0) {
        PLOG_DEBUG(REC_TAG, "frame data is empty.");
        return;
    }

    const std::string &fileID = m_currTask[IDX_V].thumb_id;
    const std::string &filePath = m_currTask[IDX_V].thumb_path;
    uint32_t duration = m_currTask[IDX_V].duration; //add by ltl 2020-02-19

    fp_save = fopen(filePath.c_str(), "wb");
    fwrite(&frame->data[0], 1, frame->data.size(), fp_save);
    fclose(fp_save);
    m_subThumb.shutdown();

    if (!filePath.empty()) {
        struct stat statbuf;
        if (0 != stat(filePath.c_str(), &statbuf)) {
            perror("stat");
            PLOG_ERROR(REC_TAG, "stat thumb file error: %s", filePath.c_str());
            goto OUT;
        }
        if (statbuf.st_size < JPG_MIN_SIZE) {
            PLOG_WARN(REC_TAG, "Drop too small file: %s, size: %ld", filePath.c_str(), statbuf.st_size);
            if (remove(filePath.c_str())) perror("remove thumb");
            goto OUT;
        }
        PLOG_INFO(REC_TAG, "INSERT ITEM: %s, %s, %d, %d, %ld, %ld", fileID.c_str(),
            filePath.c_str(),
            0,
            record::RECORD_TYPE_THUMB,
            statbuf.st_size,
            statbuf.st_atim.tv_sec);
        int rc = m_dbHelper.insertItem(fileID,
            filePath,
           //0,
            static_cast<int>(duration),
            record::RECORD_TYPE_THUMB,
            statbuf.st_size,
            statbuf.st_atim.tv_sec);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG, "INSERT DB ERROR CODE: %d", rc);
            if (remove(filePath.c_str())) perror("remove thumb");
        }
    }
OUT:
    ROS_DEBUG("thumbCallback done");
}

void RecorderManager::resetCurrTask(int idx)
{
    plt_assert(idx < TASK_NUM);
    m_currTask[idx].file_id.clear();
    m_currTask[idx].file_path.clear();
    m_currTask[idx].thumb_id.clear();
    m_currTask[idx].thumb_path.clear();
    m_currTask[idx].isUnlimited = false;
    m_currTask[idx].cntUnlimited = 0;

    m_currTask[idx].reqStart.type = 0;
    m_currTask[idx].reqStart.mode = 0;
    m_currTask[idx].reqStart.duration = 0;
    m_currTask[idx].reqStart.count = 0;
}

void RecorderManager::printCurrTask()
{
    bool taskRun = false;
    for (int i = 0; i < TASK_NUM; i++) {
        if (!m_currTask[i].file_id.empty()) {
            PLOG_DEBUG(REC_TAG, "running[%d] >>> file_id: %s, count: %d, dur: %d, mode: %d, type: %d", i,
                m_currTask[i].file_id.c_str(),
                m_currTask[i].reqStart.count,
                m_currTask[i].reqStart.duration,
                m_currTask[i].reqStart.mode,
                m_currTask[i].reqStart.type);
            taskRun = true;
        }
    }
    if (!taskRun) {
        PLOG_DEBUG(REC_TAG, "recorder idle.");
    }
}

void RecorderManager::pubCurrTaskStatus()
{
    vector<int32_t> vct;
    for (int i = 0; i < TASK_NUM; i++) {
        vct.push_back(m_currTask[i].reqStart.type);
        vct.push_back(m_currTask[i].reqStart.mode);
        vct.push_back(m_currTask[i].reqStart.duration);
        vct.push_back(m_currTask[i].reqStart.count);
    }
    mStatusPub->pubStatus(vct);
}

int RecorderManager::typeMap(int msgtype)
{
    switch (msgtype)
    {
    case record::RECORD_TYPE_RECORD:
        return IDX_V;
    case record::RECORD_TYPE_SNAPSHOT:
        return IDX_S;
    default:
        ROS_ERROR("Not supported type: %d\n", msgtype);
        return -1;
    }
}

bool RecorderManager::isDupReqStart(record_start::Request& request)
{
    return (m_currTask[IDX_V].reqStart.type == request.type
        && m_currTask[IDX_V].reqStart.mode == request.mode
        && m_currTask[IDX_V].reqStart.duration == request.duration
        //&& m_currTask[IDX_V].reqStart.count == request.count  //new request will change current count
        ) || (
            m_currTask[IDX_S].reqStart.type == request.type
        );
}

bool RecorderManager::recordStartCB(record_start::Request& request, record_start::Response& response)
{
    if (record::RECORD_TYPE_SCHED_RECORD == request.type){
        mSchedRecord = true;
        request.type  = record::RECORD_TYPE_RECORD;
    }
    PLOG_INFO(REC_TAG, "%s Request type: %d, duration: %d, count: %d", __FUNCTION__, request.type, request.duration, request.count);
    int taskType = typeMap(request.type);
    if (taskType < 0) {
        response.id = "";
        response.status = 0;
        return false;
    }
    /***************start add by ltl 2021-04-07**********************/
    if (record::RECORD_TYPE_SNAPSHOT == request.type){
        goto record;
    }
    /***************end add by ltl 2021-04-07**********************/

    if (isDupReqStart(request)) {
        PLOG_WARN(REC_TAG, "duplicate start request");
        response.id = m_currTask[taskType].file_id;
        response.status = 0;
        return true;
    }

    if (request.type == record::RECORD_TYPE_SNAPSHOT) {
        if (request.count != 1 && request.duration < SNAPSHOT_MIN_INTERVAL) {
            PLOG_WARN(REC_TAG, "snapshot duration(interval) must >= %d.", SNAPSHOT_MIN_INTERVAL);
            response.id = "";
            response.status = 0;
            return false;
        }
    } else if (request.type == record::RECORD_TYPE_RECORD) {
        if (request.duration < VIDEO_MIN_DURATION) {
            PLOG_WARN(REC_TAG, "video duration must >= %d.", VIDEO_MIN_DURATION);
            response.id = "";
            response.status = 0;
            return false;
        }
    }

    if (diskIsChecking()) {
        PLOG_WARN(REC_TAG, "disk check is running. Try again later.");
        response.id = "";
        response.status = 0;
        return false;
    }

record:
    char idStr[FILE_ID_LEN + 4];
    rand_str(idStr, FILE_ID_LEN);
    string fileId = idStr;

    pqueue_msg msg;
    msg.msg = MSG_RECORD_START;
    rec_request_t *recRequest = new rec_request_t;
    recRequest->file_id = fileId;
    recRequest->reqStart = request;
    msg.parc = recRequest;
    pqueue_push(m_queue, msg, 1);

    // startRecordMp4(recRequest, "");
    // delete recRequest;

    response.id = fileId;
    response.status = 0;
    return true;
}

bool RecorderManager::recordStopCB(record_stop::Request& request, record_stop::Response& response)
{
    PLOG_INFO(REC_TAG, "%s Request type: %d", __FUNCTION__, request.type);

    if (record::RECORD_TYPE_SCHED_RECORD == request.type){
        if (!mSchedRecord){
            return false;
        }
        request.type  = record::RECORD_TYPE_RECORD;
    }
    mSchedRecord = false;

    int taskType = typeMap(request.type);
    if (taskType < 0) {
        return false;
    }
    pqueue_msg msg;
    msg.msg = MSG_RECORD_STOP;
    msg.pars = request.type;
    pqueue_push(m_queue, msg, 1);

    //stopRecordMp4(true);

    response.status = 0;
    return true;
}

bool RecorderManager::recordGetFilesCB(record_get_files::Request& request, record_get_files::Response& response)
{
    PLOG_INFO(REC_TAG, "%s Request type: %d, id: %s, start: %d, size: %d", __FUNCTION__, request.type, request.id.c_str(), request.start, request.size);
    bool retok = false;
    if (request.id.empty()) {
        retok = m_dbHelper.queryTypeList(request.type, request.start, request.size, response.files);
    } else {
        retok = m_dbHelper.queryTypeList(request.type, request.id, request.size, response.files);
    }
    if (!retok) {
        PLOG_ERROR(REC_TAG, "%s query list of type failed.", __FUNCTION__);
        return false;
    }
    return true;
}

bool RecorderManager::recordGetFilePathCB(record_get_file_path::Request& request, record_get_file_path::Response& response)
{
    PLOG_INFO(REC_TAG, "%s Request id: %s", __FUNCTION__, request.id.c_str());

    response.path = m_dbHelper.queryFilePath(request.id);
    if (response.path.empty()) {
        PLOG_WARN(REC_TAG, "%s query file path failed by id: %s", __FUNCTION__, request.id.c_str());
        return false;
    }
    return true;
}

bool RecorderManager::recordGetFileNumCB(record_get_file_num::Request& request, record_get_file_num::Response& response)
{
    PLOG_INFO(REC_TAG, "%s Request type: %d", __FUNCTION__, request.type);

    response.count = m_dbHelper.queryTypeNum(request.type);
    if (response.count < 0) {
        PLOG_ERROR(REC_TAG, "%s query num of type failed.", __FUNCTION__);
        return false;
    }
    return true;
}

bool RecorderManager::recordDeleteFileCB(record_delete_file::Request& request, record_delete_file::Response& response)
{
    PLOG_INFO(REC_TAG, "%s Request id: %s", __FUNCTION__, request.id.c_str());
    response.status = -1;
    string filepath = m_dbHelper.queryFilePath(request.id);
    if (filepath.empty()) {
        PLOG_WARN(REC_TAG, "%s query file path failed by id: %s", __FUNCTION__, request.id.c_str());
        return false;
    }
    if (!m_dbHelper.deleteItem(request.id)) {
        PLOG_ERROR(REC_TAG, "%s delete db item failed: %s", __FUNCTION__, request.id.c_str());
        return false;
    }
    if (remove(filepath.c_str()) != 0) {
        PLOG_ERROR(REC_TAG, "%s remove file failed: %s", __FUNCTION__, filepath.c_str());
        return false;
    }

    if (filepath.rfind(".mp4") != string::npos) {
        string thumbId = filepath.substr(filepath.rfind("_") + 1, FILE_ID_LEN);
        PLOG_INFO(REC_TAG, "%s Delete thumb id: %s", __FUNCTION__, thumbId.c_str());

        string thumbPath = m_dbHelper.queryFilePath(thumbId);
        if (thumbPath.empty()) {
            PLOG_WARN(REC_TAG, "%s query file path failed by id: %s", __FUNCTION__, thumbId.c_str());
            return false;
        }
        if (!m_dbHelper.deleteItem(thumbId)) {
            PLOG_ERROR(REC_TAG, "%s delete db item failed: %s", __FUNCTION__, thumbId.c_str());
            return false;
        }
        if (remove(thumbPath.c_str()) != 0) {
            PLOG_ERROR(REC_TAG, "%s remove file failed: %s", __FUNCTION__, thumbPath.c_str());
            return false;
        }
    }

    response.status = 0;
    return true;
}

bool RecorderManager::recordGetStatusCB(record_get_status::Request& request, record_get_status::Response& response)
{
    PLOG_INFO(REC_TAG, "%s Request type: %d", __FUNCTION__, request.type);
    int taskType = typeMap(request.type);
    if (taskType < 0) {
        return false;
    }

    if (m_currTask[taskType].file_id.empty())
        response.status = 0;
    else
        response.status = 1;
    return true;
}

bool RecorderManager::recordCleanCB(record_clean::Request& request, record_clean::Response& response){
  PLOG_INFO(REC_TAG, "recordCleanCB type:%d", request.type);

  m_dbHelper.closeAndReload();

  return true;
}
}