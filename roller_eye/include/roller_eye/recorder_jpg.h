#ifndef __ROLLER_EYE_RECORDER_JPG_H__
#define __ROLLER_EYE_RECORDER_JPG_H__

#include <ros/ros.h>
#include "roller_eye/frame.h"
#include <condition_variable>
#include <thread>


namespace roller_eye {


class RecorderJPG
{
public:
    RecorderJPG(std::string fileName);
    ~RecorderJPG();
    void jpgCallback(roller_eye::frameConstPtr frame);
    void startLoop();
    void stopLoop();
    void setTrigger(bool flag);
    bool haveData();
private:
    void saveLoop();
    static const char* REC_TAG;
    std::thread mThread;
    std::mutex m_mutex;
    std::condition_variable m_cond;
    bool mShotTrigger;
    std::vector<uint8_t> mShotData;
    std::string mFileName;
};


} // namespace roller_eye

#endif //__ROLLER_EYE_RECORDER_JPG_H__