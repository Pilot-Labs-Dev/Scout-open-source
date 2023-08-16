#include <ros/ros.h>
#include "roller_eye/video_stream_pub.h"
#include "roller_eye/graphic_utils.h"

#ifdef APP_ARCH_X86
#include "roller_eye/encoder_h264.h"
#else
#include "roller_eye/encoder_h264_mpp.h"
#endif

using namespace std;

namespace roller_eye {

VideoStreamPub::VideoStreamPub(ros::NodeHandle& nh, CameraHandle *ch) :
subRefNum(0), m_pubRunning(false), mCameraHandle(ch), vsHandle(nullptr), mSessionH264(1000),
mPubH264(nh, "h264", 100, bind(&VideoStreamPub::subConnectH264,this,placeholders::_1), bind(&VideoStreamPub::subDisconnectH264,this,placeholders::_1))
{
  mNeedResume = false;
  ch->registParamListener(std::bind(&VideoStreamPub::onParamChanged, this, placeholders::_1, placeholders::_2));

  StreamInfo param = ch->getCaptureParam().sInfo;
#ifdef APP_ARCH_X86
  mEncoder = std::make_shared<EncoderH264>(&param);
#else
  mEncoder = std::make_shared<EncoderH264MPP>(&param);
#endif
  initBldFrame(param.fInfo.fmt);
}

VideoStreamPub::~VideoStreamPub()
{
  subRefNum = 0;
  m_pubRunning = false;

  if (vsHandle != nullptr) {
    video_stream_destory(vsHandle);
    vsHandle = nullptr;
  }
  mCameraHandle = nullptr;
  mEncoder = nullptr;
  if (mBldFrame.data != nullptr) {
    delete[] mBldFrame.data;
    mBldFrame.data = nullptr;
  }
}

void VideoStreamPub::onParamChanged(CaputreParam *param, bool resume)
{
  initBldFrame(param->sInfo.fInfo.fmt);
  if ((!resume && !m_pubRunning) ||
     (resume && !mNeedResume)) {
    mNeedResume = false;
    if (mEncoder.unique()) {
    #ifdef APP_ARCH_X86
      mEncoder.reset(new EncoderH264(&param->sInfo));
    #else
      mEncoder.reset(new EncoderH264MPP(&param->sInfo));
    #endif
    } else {
      mEncoder.reset();
    }
    return;
  }

  if (!resume){
    mNeedResume = true;
    stopPublish();
    video_stream_destory(vsHandle);
  }else {
    mNeedResume = false;
    mSessionH264++;

    if (mEncoder.unique()) {
  #ifdef APP_ARCH_X86
      mEncoder.reset(new EncoderH264(&param->sInfo));
  #else
      mEncoder.reset(new EncoderH264MPP(&param->sInfo));
  #endif
    } else {
      mEncoder.reset();
    }

    vsHandle = mCameraHandle->createVideoStream();
    startPublish();
  }
}

CaputreParam VideoStreamPub::getCaptureParam(void) {
  return mCameraHandle->getCaptureParam();
}

int VideoStreamPub::getNumSubscribers() {
  return subRefNum;
}

FrameBuff *VideoStreamPub::getFrame() {
  return (subRefNum > 0) ? video_stream_get_frame(vsHandle) : nullptr;
}

void VideoStreamPub::objCallback(const detect& msg)
{
  V_STREAM_VERBOSE("objCallback: %u, score: %f, name: %s. [%f, %f, %f, %f]\n", msg.seq, msg.score, msg.name.c_str(),
     msg.top, msg.left, msg.bottom, msg.right);
  int margin=2;//yuyvDrawRect/yuv420pDrawRect may cause overflow
  int maxW=getCaptureParam().sInfo.fInfo.width-margin;
  int maxH=getCaptureParam().sInfo.fInfo.height-margin;

  mObjDet.top = msg.top < margin ? margin:msg.top;
  mObjDet.left = msg.left < margin ? margin:msg.left;
  mObjDet.bottom = msg.bottom > maxH? maxH : msg.bottom;
  mObjDet.right = msg.right > maxW?maxW : msg.right;
  mObjDet.score = msg.score;

  V_STREAM_VERBOSE("objCallback: [%f, %f, %f, %f]\n", mObjDet.left, mObjDet.top, mObjDet.right - mObjDet.left, mObjDet.bottom - mObjDet.top);
}

void VideoStreamPub::initBldFrame(int fmt)
{
  FILE *fp = nullptr;
  if (fmt == V4L2_PIX_FMT_YUYV) {
    fp = fopen(BLEND_FILE_YUYV.c_str(), "rb");
  } else if (fmt == V4L2_PIX_FMT_YUV420) {
    fp = fopen(BLEND_FILE_YUV420P.c_str(), "rb");
  }
  if (!fp) {
    ROS_ERROR("Open blending file failed.\n");
    return;
  }
  fseek(fp, 0, SEEK_END);
  int size = ftell(fp);
  if (size <= 0) {
    ROS_ERROR("Empty file.\n");
    return;
  }
  if (mBldFrame.data != nullptr) {
    delete[] mBldFrame.data;
    mBldFrame.data = nullptr;
  }
  mBldFrame.data = new uint8_t[size];
  fseek(fp, 0, SEEK_SET);
  int ret = fread(mBldFrame.data, 1, size, fp);
  fclose(fp);
  if (ret != size) {
    delete[] mBldFrame.data;
    mBldFrame.data = nullptr;
  }
}

void VideoStreamPub::subConnectH264(int cnt)
{
  ROS_DEBUG("subConnectH264 subscriber cnt: %d", cnt);
  subRefNum = cnt;
  if (subRefNum > 0) {
    if (m_pubRunning == false) {
      vsHandle = mCameraHandle->createVideoStream();
      startPublish();
    } else {
      ROS_DEBUG("video stream already running.");
    }
  }
}

void VideoStreamPub::subDisconnectH264(int cnt)
{
  ROS_DEBUG("subDisconnectH264 subscriber cnt: %d", cnt);
  subRefNum = cnt;
  if (subRefNum == 0) {
    if (m_pubRunning == true) {
      stopPublish();
      video_stream_destory(vsHandle);
      vsHandle = nullptr;
      ROS_DEBUG("subDisconnectH264 end");
    } else {
      ROS_DEBUG("video stream already stop.");
    }
  }
}


void VideoStreamPub::startPublish()
{
  m_pubRunning = true;
  m_pubThread = std::thread(&VideoStreamPub::publishLoop, this);
}

void VideoStreamPub::stopPublish()
{
  if(!m_pubRunning) {
    return;
  }

  m_pubRunning = false;
  m_pubThread.join();
}

void VideoStreamPub::publishLoop()
{
  FrameBuff *frameBuff;
  Frame *tmpFrame = new Frame();
  struct timespec tpTime;
  uint64_t timeStamp = 0;
  int frameW = getCaptureParam().sInfo.fInfo.width;
  int frameH = getCaptureParam().sInfo.fInfo.height;
  int fmt = getCaptureParam().sInfo.fInfo.fmt;

  while (m_pubRunning) {
    frameBuff = getFrame();
    if (frameBuff == nullptr) {
      usleep(100*1000);
      continue;
    }

    clock_gettime(CLOCK_MONOTONIC, &tpTime);
    timeStamp = tpTime.tv_sec*1000 + tpTime.tv_nsec/1000000;

    if (mBldFrame.data != nullptr) {
      if (fmt == V4L2_PIX_FMT_YUYV) {
        GraphicUtils::yuyv422Blend((uint8_t*)frameBuff->addr, frameW, frameH, mBldFrame.data, mBldFrame.width, mBldFrame.height, 0, frameH - mBldFrame.height, ALPHA_VAL);
      } else if (fmt == V4L2_PIX_FMT_YUV420) {
        GraphicUtils::yuv420pBlend((uint8_t*)frameBuff->addr, frameW, frameH, mBldFrame.data, mBldFrame.width, mBldFrame.height, 0, frameH - mBldFrame.height, ALPHA_VAL);
      }
    }
    if (mObjDet.score > 0) {
      int left,right,top,bottom;
      left=mObjDet.left;
      right=mObjDet.right;
      top=mObjDet.top;
      bottom=mObjDet.bottom;
      if(GraphicUtils::checkRectange(frameW,frameH,left,right,top,bottom)){//objCallback has no lock,need to check
        if (fmt == V4L2_PIX_FMT_YUYV) {
          GraphicUtils::yuyvDrawRect((uint8_t*)frameBuff->addr, frameW, left, top, right - left, bottom - top, 255, 0, 0);
        } else if (fmt == V4L2_PIX_FMT_YUV420) {
          GraphicUtils::yuv420pDrawRect((uint8_t*)frameBuff->addr, frameW, frameH, left, top, right - left, bottom - top, 255, 0, 0);
        }
      }
    }
    tmpFrame->setData((uint8_t*)frameBuff->addr,frameBuff->size,timeStamp);

    publishH264(tmpFrame);

    video_stream_return_frame(vsHandle, frameBuff);

  }
  ROS_DEBUG("exit publishLoop\n");
  delete tmpFrame;
}

void VideoStreamPub::publishH264(Frame *tmpFrame)
{
  static uint32_t seq = 0;
  static uint32_t oseq = 0;

  static int counter = 0;
  static int totalKb = 0;

  roller_eye::frame msgFrame;

  int ret = mEncoder->put_frame(tmpFrame);
  if (ret<0){
    ROS_DEBUG("put_frame error %d\n", ret);
  }

  oseq++;
  auto ptrEncodedFrame = mEncoder->encode_frame();
  auto ptrVideoFrame = dynamic_cast<const VideoFrame*>(ptrEncodedFrame);
  if (ptrVideoFrame != nullptr) {
    msgFrame.seq = ++seq;
    msgFrame.oseq = oseq;
    msgFrame.session = mSessionH264;
    msgFrame.stamp = tmpFrame->getTimestamp();
    msgFrame.type = msgFrame.VIDEO_STREAM_H264;
    msgFrame.par1 = getCaptureParam().sInfo.fInfo.width;
    msgFrame.par2 = getCaptureParam().sInfo.fInfo.height;
    msgFrame.par3 = ptrVideoFrame->isKeyframe;
    msgFrame.data.clear();

    if (counter++ < 50){
      totalKb += ptrVideoFrame->getSize();
    }else{
      if ( totalKb/100 < 500){  //less than 500
          ROS_DEBUG("h264 data size is not correct %d\n", totalKb/100);
          exit(-1);
      }
      counter = 0;
      totalKb = 0;
    }

    msgFrame.data.insert(msgFrame.data.end(), &ptrVideoFrame->getData()[0], &ptrVideoFrame->getData()[ptrVideoFrame->getSize()]);

    V_STREAM_VERBOSE("publish264: %u, stamp: %ld, datasize: %lu, key: %d\n", msgFrame.seq, msgFrame.stamp, msgFrame.data.size(), msgFrame.par3);

    mPubH264.publish(msgFrame);
  }
}

} // namespace roller_eye
