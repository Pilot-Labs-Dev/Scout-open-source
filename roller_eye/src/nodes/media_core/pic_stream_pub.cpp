#include <ros/ros.h>
#include "roller_eye/pic_stream_pub.h"

#include "roller_eye/encoder_jpeg.h"


using namespace std;

namespace roller_eye {

PicStreamPub::PicStreamPub(ros::NodeHandle& nh, CameraHandle *ch) :
subRefNum(0), m_pubRunning(false), mCameraHandle(ch), vsHandle(nullptr), mSessionJPG(3000),
mPubJPG(nh, "jpg", 100, bind(&PicStreamPub::subConnectJPG,this,placeholders::_1), bind(&PicStreamPub::subDisconnectJPG,this,placeholders::_1))
{
  mNeedResume  = false;
  ch->registParamListener(std::bind(&PicStreamPub::onParamChanged, this, placeholders::_1, placeholders::_2));
  StreamInfo param = ch->getCaptureParam().sInfo;
  mEncoder.reset(new EncoderJPEG(&param));
}

PicStreamPub::~PicStreamPub()
{
  subRefNum = 0;
  m_pubRunning = false;

  if (vsHandle != nullptr) {
    video_stream_destory(vsHandle);
    vsHandle = nullptr;
  }
  mCameraHandle = nullptr;
  mEncoder = nullptr;
}

void PicStreamPub::onParamChanged(CaputreParam *param, bool resume)
{
  if ((!resume && !m_pubRunning)||
        (resume && !mNeedResume)) {
    mNeedResume = false;
    mEncoder.reset(new EncoderJPEG(&param->sInfo));
    return;
  }

  if (!resume){
    mNeedResume = true;
    stopPublish();
    video_stream_destory(vsHandle);
  }else {
    mNeedResume = false;
    mSessionJPG++;
    mEncoder.reset(new EncoderJPEG(&param->sInfo));
    vsHandle = mCameraHandle->createVideoStream(1);
    startPublish();
  }
}

CaputreParam PicStreamPub::getCaptureParam(void) {
  return mCameraHandle->getCaptureParam();
}

int PicStreamPub::getNumSubscribers() {
  return subRefNum;
}

FrameBuff *PicStreamPub::getFrame() {
  return (subRefNum > 0) ? video_stream_get_frame(vsHandle) : nullptr;
}


void PicStreamPub::subConnectJPG(int cnt)
{
  ROS_DEBUG("subConnectJPG subscriber cnt: %d", cnt);
  subRefNum = cnt;
  if (subRefNum > 0) {
    if (m_pubRunning == false) {
      vsHandle = mCameraHandle->createVideoStream(1);
      startPublish();
    } else {
      ROS_DEBUG("video stream already running.");
    }
  }
}

void PicStreamPub::subDisconnectJPG(int cnt)
{
  ROS_DEBUG("subDisconnectJPG subscriber cnt: %d", cnt);
  subRefNum = cnt;
  if (subRefNum == 0) {
    if (m_pubRunning == true) {
      stopPublish();
      video_stream_destory(vsHandle);
      vsHandle = nullptr;
    } else {
      ROS_DEBUG("video stream already stop.");
    }
  }
}


void PicStreamPub::startPublish()
{
  m_pubRunning = true;
  m_pubThread = std::thread(&PicStreamPub::publishLoop, this);
}

void PicStreamPub::stopPublish()
{
  if(!m_pubRunning) {
    return;
  }

  m_pubRunning = false;
  m_pubThread.join();
}

void PicStreamPub::publishLoop()
{
  FrameBuff *frameBuff;
  Frame *tmpFrame = new Frame();
  struct timespec tpTime;
  uint64_t timeStamp = 0;

  while (m_pubRunning) {
    frameBuff = getFrame();
    if (frameBuff == nullptr) {
      usleep(100*1000);
      continue;
    }

    clock_gettime(CLOCK_MONOTONIC, &tpTime);
    timeStamp = tpTime.tv_sec*1000 + tpTime.tv_nsec/1000000;

    tmpFrame->setData((uint8_t*)frameBuff->addr,frameBuff->size,timeStamp);

    publishJPG(tmpFrame);

    video_stream_return_frame(vsHandle, frameBuff);
  }

  delete tmpFrame;
}

void PicStreamPub::publishJPG(Frame *tmpFrame)
{
  static uint32_t seq = 0;
  roller_eye::frame msgFrame;

  mEncoder->put_frame(tmpFrame);
  auto ptrEncodedFrame = mEncoder->encode_frame();
  auto ptrPicFrame = dynamic_cast<const PictureFrame*>(ptrEncodedFrame);
  if (ptrPicFrame != nullptr) {
    msgFrame.seq = ++seq;
    msgFrame.session = mSessionJPG;
    msgFrame.stamp = tmpFrame->getTimestamp();
    msgFrame.type = msgFrame.VIDEO_STREAM_JPG;
    msgFrame.par1 = getCaptureParam().sInfo.fInfo.width;
    msgFrame.par2 = getCaptureParam().sInfo.fInfo.height;
    msgFrame.par3 = 0;
    msgFrame.data.clear();
    msgFrame.data.insert(msgFrame.data.end(), &ptrPicFrame->getData()[0], &ptrPicFrame->getData()[ptrPicFrame->getSize()]);

    PIC_STREAM_VERBOSE("publishJPG: %u, stamp: %ld, datasize: %lu\n", msgFrame.seq, msgFrame.stamp, msgFrame.data.size());

    mPubJPG.publish(msgFrame);
  }
}

} // namespace roller_eye
