#include "roller_eye/recorder_jpg.h"
#include "plog_plus.h"

namespace roller_eye {

const char* RecorderJPG::REC_TAG = "rec_jpg";

RecorderJPG::RecorderJPG(std::string filename) : mFileName(filename)
{
}

RecorderJPG::~RecorderJPG()
{
    PLOG_INFO(REC_TAG, "~RecorderJPG");
    mShotTrigger = false;
    mShotData.clear();
    mFileName.clear();
}

void RecorderJPG::startLoop()
{
    mShotTrigger = false;
	mThread = std::thread(&RecorderJPG::saveLoop, this);
}


bool RecorderJPG::haveData()
{
    int nSize = mShotData.size();
    return nSize>0;
}

void RecorderJPG::stopLoop()
{
    if (mThread.joinable()) {
        mThread.join();
    }
}

void RecorderJPG::saveLoop()
{
    PLOG_INFO(REC_TAG, "RecorderJPG::saveLoop");
    FILE *fp_save = nullptr;
	while (1) {
        //PLOG_INFO(REC_TAG, "jpgSaveLoop wait start");
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cond.wait(lock, [this]() {return mShotTrigger;});

        if (mShotData.size()) {
            fp_save = fopen(mFileName.c_str(), "wb");
            fwrite(&mShotData[0], 1, mShotData.size(), fp_save);
            fclose(fp_save);
            break;
        } else {
            PLOG_INFO(REC_TAG, "mShotData is empty.");
        }
        break;
	}
    PLOG_DEBUG(REC_TAG, "RecorderJPG::saveLoop exit.");
}

void RecorderJPG::setTrigger(bool flag)
{
    mShotTrigger = flag;
	m_cond.notify_one();
}

void RecorderJPG::jpgCallback(roller_eye::frameConstPtr frame)
{
#if 1
	struct timespec tpTime;
    clock_gettime(CLOCK_MONOTONIC, &tpTime);
    uint64_t nowTime = tpTime.tv_sec*1000 + tpTime.tv_nsec/1000000;
    ROS_DEBUG("jpgCallback: %u, stamp: %ld, datasize: %lu, timediff: %ld", frame->seq, frame->stamp, frame->data.size(), nowTime - frame->stamp);
#endif
    std::unique_lock<std::mutex> locker(m_mutex);
    if (mShotData.empty()){
        mShotData.insert(mShotData.end(), &frame->data[0], &frame->data[frame->data.size()]);
    }
}

} // namespace roller_eye
