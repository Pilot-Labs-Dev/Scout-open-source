#include"camera_handle.hpp"
#include"graphic_utils.h"
#include"plt_assert.h"
#include"plt_config.h"

namespace roller_eye {
static bool s_grey_mode=false;
void set_image_grey_mode(bool grey)
{
  s_grey_mode=grey;
}
#ifndef APP_ARCH_X86
static int CAMERA_FLIP_TYPE=-1;
extern "C"  void rgaCopyAndFlip(void* dst,void* src,int len,struct _FrameInfo* info,void *priv);
extern "C"  void video_stream_cb(int evt,void *priv);

int convert2RKfmt(uint32_t fmt)
{
  switch(fmt){
    case V4L2_PIX_FMT_YUV420:
      return RK_FORMAT_YCbCr_420_SP;
    break;
    default:
     break;
  }
  return RK_FORMAT_UNKNOWN;
}
//#define DEBUG_DUMP_RAW_IMG_DATA
#ifdef DEBUG_DUMP_RAW_IMG_DATA
static void saveImage(void* data,int len,const char* path)
{
  FILE *file=fopen(path,"wb");
  if(file!=NULL){
    fwrite(data,1,len,file);
    fclose(file);
  }
}
static void dumpRawImg(void*src,void* dst,int len)
{
    static int frameCnt=0;
    if(++frameCnt==300){
      saveImage(src,len,"/tmp/src.yuv420");
      saveImage(dst,len,"/tmp/dst.yuv420");
    }
}
#endif
void rgaCopyAndFlip(void* dst,void* src,int len,struct _FrameInfo* info,void *priv)
{
  static const int MAX_BUF_SIZE=1920*1080/4;
  // static char buf[MAX_BUF_SIZE];

  int fmt=convert2RKfmt(info->fmt);
  int greySize=info->width*info->height;
  int uvLen=greySize/4;
  plt_assert(info->fmt==V4L2_PIX_FMT_YUV420 && uvLen<=MAX_BUF_SIZE &&  CAMERA_FLIP_TYPE>=0 && CAMERA_FLIP_TYPE<=HAL_TRANSFORM_FLIP_V);

  GraphicUtils::getInstance()->rgaConvert((uint8_t*)src,info->width,info->height,fmt,(uint8_t*)dst,info->width,info->height,fmt,CAMERA_FLIP_TYPE);
  //swap uv
  // if(CAMERA_FLIP_TYPE == HAL_TRANSFORM_FLIP_V){
  //   memcpy(buf,(char*)dst+greySize,uvLen);
  //   memcpy((char*)dst+greySize,(char*)dst+greySize+uvLen,uvLen);
  //   memcpy((char*)dst+greySize+uvLen,buf,uvLen);
  // }

  if( s_grey_mode){
    memset((char*)dst+greySize,128,len-greySize);
  }
#ifdef DEBUG_DUMP_RAW_IMG_DATA
  dumpRawImg(src,dst,len);
#endif
}
void video_stream_cb(int evt,void *priv)
{
  if(CAMERA_FLIP_TYPE<0){
    DeviceDefaultConfig cfg;
    CAMERA_FLIP_TYPE=cfg.getCameraFlipType();
  }
  CameraHandle::getInstance()->sendVideoStreamEvent(evt);
}
#endif
void CameraHandle::newCameraHandle(int w, int h, int frm_rate_num, int frm_rate_den,uint32_t fmt, int h264Quality, int cameraLight)
{
  if (!isNewParam(w, h, frm_rate_num, frm_rate_den,fmt,h264Quality, cameraLight)) {
    return;
  }

  for (auto cb : m_parListeners) {
    cb(&m_param,false);
  }

  m_mutex.lock();
  if (cmHandle != nullptr) {
    video_stream_destory_camera(cmHandle);
    cmHandle = nullptr;
  }

  m_param.sInfo.fInfo.width = w;
  m_param.sInfo.fInfo.height = h;
  m_param.sInfo.fpsDen = frm_rate_den;
  m_param.sInfo.fpsNum=frm_rate_num;
  m_param.sInfo.fInfo.fmt = fmt;
  m_param.sInfo.h264Quality = h264Quality;
  m_param.sInfo.cameraLight = cameraLight;

  cmHandle = camera_handle_create(&m_param);
  // vsHandle = video_stream_create(cmHandle, 0);
  m_mutex.unlock();

  for (auto cb : m_parListeners) {
    cb(&m_param,true);
  }

}

VSHandle CameraHandle::createVideoStream(int buffCnt)
{
  std::lock_guard<std::mutex> lock(m_mutex);
	return video_stream_create(cmHandle,buffCnt);
}

CaputreParam CameraHandle::getCaptureParam(void)
{
  return m_param;
}

void CameraHandle::registParamListener(ParamListener recv)
{
  m_parListeners.push_back(recv);
}
void CameraHandle::registVideoStreamEventCB(const VideoStreamEventCB& cb)
{
  mEvt=cb;
}
void CameraHandle::sendVideoStreamEvent(int evt)
{
  if(mEvt){
    mEvt(evt);
  }
}
CameraHandle::~CameraHandle()
{
  if (cmHandle != nullptr) {
    video_stream_destory_camera(cmHandle);
    cmHandle = nullptr;
  }
  memset(&m_param, 0, sizeof(CaputreParam));
  m_parListeners.clear();
}
CameraHandle::CameraHandle()
{
  cmHandle = nullptr;
  memset(&m_param, 0, sizeof(CaputreParam));
  m_parListeners.clear();
}

CMHandle CameraHandle::camera_handle_create(CaputreParam *param)
{
  CaputreParam prm;
  CMHandle m_camera;

  memset(&prm,0,sizeof(prm));
  //if (m_camera == NULL) {
    prm.sInfo.fInfo.width = param->sInfo.fInfo.width;
    prm.sInfo.fInfo.height = param->sInfo.fInfo.height;
    prm.sInfo.fInfo.fmt = param->sInfo.fInfo.fmt; //V4L2_PIX_FMT_YUYV;
    prm.sInfo.fpsDen = param->sInfo.fpsDen;
    prm.sInfo.fpsNum = param->sInfo.fpsNum;
    prm.sInfo.h264Quality = param->sInfo.h264Quality;
    prm.sInfo.cameraLight = param->sInfo.cameraLight;
    prm.defaultCount=3;
    prm.buffCount=6;

    prm.v4l2BuffCount=3;
    prm.devIdx=0;
#ifndef APP_ARCH_X86
    if(prm.sInfo.fInfo.fmt==V4L2_PIX_FMT_YUV420){//hardware accellerator
      prm.hooks.cp=rgaCopyAndFlip;
      prm.hooks.ev=video_stream_cb;
    }
#endif
    //strcpy(prm.videoDevPath,"/dev/v4l-subdev2");
    strcpy(prm.videoDevPath,"/dev/video0");

    m_camera = video_stream_create_camera(&prm);
  //}

  video_stream_get_camera_param(m_camera, param);
  return m_camera;
}

bool CameraHandle::isNewParam(int w, int h, int frm_rate_num, int frm_rate_den,uint32_t fmt, int h264Quality, int cameraLight)
{
  return m_param.sInfo.fInfo.width != w
    || m_param.sInfo.fInfo.height != h
    || m_param.sInfo.fInfo.fmt != fmt
    || m_param.sInfo.fpsDen != frm_rate_den
    || m_param.sInfo.fpsNum!= frm_rate_num
    || m_param.sInfo.h264Quality !=h264Quality
    || m_param.sInfo.cameraLight !=cameraLight;
}
} // namespace roller_eye