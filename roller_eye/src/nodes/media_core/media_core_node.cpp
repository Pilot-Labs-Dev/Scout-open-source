#include<mutex>
#include <ros/ros.h>
#include"sensor_msgs/Illuminance.h"
#include"roller_eye/plterrno.h"
#include "roller_eye/audio_stream_pub.h"
#include "roller_eye/video_stream_pub.h"
#include "roller_eye/obj_detect_pub.h"
#include "roller_eye/pic_stream_pub.h"
#include <dynamic_reconfigure/server.h>
#include "roller_eye/ImageConfig.h"
#include"roller_eye/data_publisher.h"
#include "alg_backing_up.h"
#include"mono_image_pub.h"
#include"roller_eye/system_define.h"
#include"roller_eye/video_stream.h"
#include"roller_eye/device_interface.h"
#include"roller_eye/plt_config.h"
#include"motion_detect.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include"roller_eye/plt_config.h"
#include "roller_eye/adjust_ligth.h"
#include "roller_eye/night_get.h"
#include "roller_eye/stop_detect.h"
#include "roller_eye/data_publisher2.h"
#include"roller_eye/cv_img_stream.h"
#include "roller_eye/graphic_utils.h"
#include"roller_eye/plog_plus.h"
#include "roller_eye/adjust_exposure_time.h"
#include "roller_eye/getDiffAngleWhenPatrol.h"
#include "roller_eye/saveTmpPicForStartPath.h"

#include "zlog.h"
#include <iostream>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
using namespace std;
#ifndef SHM_FAILED
#define SHM_FAILED -1
#endif
#define FLAG IPC_CREAT|SHM_R|SHM_W
typedef struct
{
    int id;
    int rid;
    float angle;
}recordData;
const char* shmt_path = "/tmp";

using namespace roller_eye;
using namespace std;


enum{
NIGHT_MODE_OFF=0,
NIGHT_MODE_ON,
NIGHT_MODE_AUTO
};

#define CORE_NODE_TAG      "CoreNode"

//#define CH0_SAMPLE_COUNT  20
#define CH0_SAMPLE_COUNT  30
#define MAX_IR_BRIGHTNESS  99

// #define FEATURE_DETECT_IMG_WIDTH  640
// #define FEATURE_DETECT_IMG_HEIGHT 360
#define FEATURE_DETECT_IMG_WIDTH  320
#define FEATURE_DETECT_IMG_HEIGHT 180

static shared_ptr<ObjDetectPub> g_obj_detect_ptr;
static shared_ptr<ObjDetectPub> g_pile_detect_ptr;
static shared_ptr<MotionDetect> g_motion_detect_ptr;
static  shared_ptr<DataPublisher2<roller_eye::detect,MotionDetect>> g_motion_detect_pub_ptr;
static shared_ptr<CVGreyImageStream> g_cvStream_ptr;
class InitDZLog{
    public:
    InitDZLog()
    {
      log_t arg = {
        confpath:	"/var/roller_eye/config/log/" CORE_NODE_TAG ".cfg",
        levelpath:	"/var/roller_eye/config/log/" CORE_NODE_TAG ".level",
        logpath:	"/var/log/node/" CORE_NODE_TAG ".log",
        cname:		CORE_NODE_TAG
      };
      if(0 != dzlogInit(&arg,2)){
          printf("%s log int error.\r\n",CORE_NODE_TAG);
      }
    }
    ~InitDZLog()
    {
		dzlogfInit();
    }
} ;
class NightModeProcess{
public:
  NightModeProcess(DeviceDefaultConfig&cfg):
  mMode(NIGHT_MODE_OFF),
  mIRProcessRunning(false),
  mIRActive(false),
  mAdjusting(false),
  mAdding(false),
  mPreTime(-1),
  mAdjustTime(-1),
  mCurIRValue(0),
  mChargingStatus(-1),
  mPrevCh0(0)
  {
    CAPTURE_TIME=cfg.getNightModeDuration();
    mVth1   = cfg.getNightModeVth1();
    mVth2   = cfg.getNightModeVth2();
    mBand = cfg.getNightModeBand();
    mAvgLightV = mVth2+mBand;
    mSamples = cfg.getNightModeSamples();
    mDaySamples  = cfg.getNightModeDaySamples();
    mMinCnt = cfg.getNightModeMinCnt();
    mDayCnt = cfg.getNightModeDayCnt();

    close_ir_cut();
    close_ir_led();
    if(!mLight){
      mLight=mHandle.subscribe("SensorNode/light",20,&NightModeProcess::IlluminanceCallback,this);
    }
    mTimer=mHandle.createTimer(ros::Duration(CAPTURE_TIME),&NightModeProcess::timerCallback,this);
    mTimer.stop();

   mBatteryStatus=mHandle.subscribe("SensorNode/simple_battery_status",10,
        &NightModeProcess::batteryStatusCallback,this);
  }
  void setNightMode(int mode, int nightVisionSensitivity)
  {
    lock_guard<mutex> lock(mProcessMutex);

    DeviceDefaultConfig cfg;
    if (0==nightVisionSensitivity){
      mVth1   = cfg.getNightModeVth1();
      mVth2   = cfg.getNightModeVth2();
      mBand = cfg.getNightModeBand();
      mAvgLightV = mVth2+mBand;
    }else{
      mVth1   = cfg.getNightModeVth3();
      mVth2   = cfg.getNightModeVth4();
      mBand = cfg.getNightModeBand2();
      mAvgLightV = mVth2+mBand;
    }
    mMode = mode;
    if(mIRProcessRunning){
        setupIRProcess();
    }
  }
  void start()
  {
    lock_guard<mutex> lock(mProcessMutex);
    setupIRProcess();
  }
  void stop()
  {
    lock_guard<mutex> lock(mProcessMutex);
    closeIRProcess();
  }

void batteryStatusCallback(const statusConstPtr &s)
{
    if (mChargingStatus != s->status[2] ){
        if(mIRProcessRunning){
          setupIRProcess();
        }
    }
    mChargingStatus =s->status[2];
}

  bool adjust(adjust_ligthRequest & req,adjust_ligthResponse& res)
  {
    if (!mIRActive){
      return false;
    }

    static int lastIRvalue = mCurIRValue;
    switch(req.cmd){
      case 0:{
        if (!mAdjusting){
          lastIRvalue = mCurIRValue;
        }
        mAdjusting = true;
        int value = 0.1 * mCurIRValue;
        if (mCurIRValue<6){
          value = 0.5*mCurIRValue;
        }
        value = value>1?value:1;
        PLOG_INFO(CORE_NODE_TAG,"adjust light %d ",value);
        setIRLed(value);
      }
      break;
      case 1:{
        mAdjusting = false;
        setIRLed(lastIRvalue);
        PLOG_INFO(CORE_NODE_TAG,"adjust light1 %d ",lastIRvalue);
      }
      break;
      case 2:
        mAdjusting = false;
        setIRLed(MAX_IR_BRIGHTNESS);
        PLOG_INFO(CORE_NODE_TAG,"adjust light2 %d ",MAX_IR_BRIGHTNESS);
      break;
      default:
        setIRLed(MAX_IR_BRIGHTNESS);
        mAdjusting = false;
      break;
    }
    return true;
  }

  bool nightGet(night_getRequest & req,night_getResponse& res)
  {
    res.isNight = mIRActive;
    res.brightness = mCurIRValue;
    return true;
  }

private:
void openStatic()
{
    if(!mLight){
      reset_statstic();
      mLight=mHandle.subscribe("SensorNode/light",20,&NightModeProcess::IlluminanceCallback,this);
    }
    mTimer.start();
}
void closeStastic()
{
    if (mTimer.hasStarted()){
          mTimer.stop();
    }
}
 void setupIRProcess()
  {
    PLOG_INFO(CORE_NODE_TAG,"setup ir process,mode=%d",mMode);
    if(mMode==NIGHT_MODE_OFF){
      offProc();
    }else{
      openStatic();
      if(mMode==NIGHT_MODE_ON){
        onProc();
      }
    }
    mIRProcessRunning=true;
  }
  void closeIRProcess()
  {
    offProc();
    mIRProcessRunning=false;
  }

   void timerCallback(const ros::TimerEvent& evt)
  {
    mIRMutex.lock();
    if (mIRActive){
      mAvgLightV = mAvgLightV-mAvgLightV/mSamples+mPrevCh0/mSamples;
      if (++mCnt < mMinCnt){
        mIRMutex.unlock();
        return;
      }
    }else if (mPrevCh0 == 0.0){
        mAvgLightV = mPrevCh0;
    } else{
      mAvgLightV = mAvgLightV-mAvgLightV/mDaySamples+mPrevCh0/mDaySamples;
      if (++mCnt < mDayCnt){
        mIRMutex.unlock();
        return;
      }
    }
    mIRMutex.unlock();
    if (!mIRProcessRunning){
      return;
    }
    switch(mMode){
      case NIGHT_MODE_ON:
        onProc();
      break;
      case NIGHT_MODE_AUTO:
        autoProc();
      break;
      default:
        offProc();
      break;
    }
  }

  void IlluminanceCallback(const sensor_msgs::IlluminanceConstPtr& ptr)
  {
    int illum = static_cast<int>(ptr->illuminance);
    lock_guard<mutex> lock(mProcessMutex);
    int Ch0 =  (illum>>16) & 0xFFFF;
    int Ch1 = illum & 0xFFFF;
    mPrevCh0 = static_cast<double>(Ch0);
  }

  void onProc()
  {
    enable_ir();
    set_image_grey_mode(true);
  }

  void autoProc()
  {
    if (mIRActive){
      if (mAvgLightV<(mVth2-mBand)  && !mAdjusting ){
        if (mCurIRValue < 10){
          setIRLed(mCurIRValue+1);
        }else{
          setIRLed(1.1 * mCurIRValue);
        }
      }else if (mAvgLightV>(mVth2+mBand)){
          if (0 == mCurIRValue){
            disable_ir();
            set_image_grey_mode(false);
          }else{
              setIRLed(0.9*mCurIRValue);
          }
      }
    }else if (mAvgLightV<mVth1){
      enable_ir();
      set_image_grey_mode(true);
    }
}

  void offProc()
  {
     closeStastic();
      set_image_grey_mode(false);
      disable_ir();
  }
  int setIRLed(int brightness)
  {
    if (brightness>=MAX_IR_BRIGHTNESS ){
        if (MAX_IR_BRIGHTNESS == mCurIRValue){
          return -1;
        }
        brightness = MAX_IR_BRIGHTNESS;
    }

    mIRMutex.lock();
    int ret = open_ir_led( brightness);
    reset_statstic();
    mIRMutex.unlock();
    mAdjustTime = ros::Time::now().toSec();
    mCurIRValue = brightness;
    return ret;
  }

  void reset_statstic()
  {
    mCnt=0;
  }
  void enable_ir()
  {
      if(mIRActive){
        return;
      }
      mAdjustTime = ros::Time::now().toSec();
      if(setIRLed(MAX_IR_BRIGHTNESS)<0){
        PLOG_ERROR(CORE_NODE_TAG,"open ir led fail");
      }

      if(open_ir_cut()<0){
            PLOG_ERROR(CORE_NODE_TAG,"open ir cut fail");
      }

      mIRActive=true;
      PLOG_ERROR(CORE_NODE_TAG,"ir mode enabled");
  }
  void disable_ir()
  {
      reset_statstic();
      if(!mIRActive){
        return;
      }
      if(close_ir_cut()<0){
        PLOG_ERROR(CORE_NODE_TAG,"close ir cut fail");
      }
      if(close_ir_led()<0){
        PLOG_ERROR(CORE_NODE_TAG,"close ir fail");
      }
      mIRActive=false;
      mCurIRValue = 0;
      mAdjustTime = ros::Time::now().toSec();
      PLOG_ERROR(CORE_NODE_TAG,"ir mode disabled");
  }
  ros::NodeHandle mHandle;
  ros::Subscriber mLight;
  ros::Subscriber mBatteryStatus;
  ros::Timer mTimer;

  double mPrevCh0;
  int mMode;
  int mChargingStatus;
  mutex mProcessMutex;
  mutex mIRMutex;
  bool mIRProcessRunning;
  bool mIRActive;
  bool mAdjusting;
  bool mAdding;
  int mSamples;
  int mDaySamples;
  double mVth1;
  double mVth2;
  double mBand;
  double mPreTime;
  double mAdjustTime;

  int mCnt;
  double CAPTURE_TIME;
  double mAvgLightV;
  int mCurIRValue;
  int mMinCnt;
  int mDayCnt;
};

static shared_ptr<NightModeProcess> g_night_mode_ptr;

static void videoStreamEventCB(int evt)
{
  if(!g_night_mode_ptr){
    return;
  }
  if(evt==VIDEO_STREAM_EVENT_OPEN){
    PLOG_INFO(CORE_NODE_TAG,"open night mode\n");
    g_night_mode_ptr->start();
  }else if(evt==VIDEO_STREAM_EVENT_CLOSE){
    PLOG_INFO(CORE_NODE_TAG,"close night mode\n");
    g_night_mode_ptr->stop();
  }
}

bool modifyBrightness(int brightness)
{
  bool bRet = false;
  ifstream input("/etc/iqfiles/ps5268_default_default.xml");
  if (input.good()){
      std::string line((std::istreambuf_iterator<char>(input)),    std::istreambuf_iterator<char>());
      if (!line.empty()){
          int pos = line.find("<GainDot",0);
          int pos1 = line.find("</GainDot>",0);
          if (pos>0 && pos1>pos){
            input.close();
            string s1 = line.substr(0,pos);
            string s2 = line.substr(pos1,line.length()-pos1);
            string s = line.substr(pos,pos1-pos);
            string key="<GainDot index=\"1\" type=\"double\" size=\"[1 6]\">\r                  [1 1 2 4 4 %d]\r               ";
            char buf[256]={0};
            sprintf(buf,key.c_str(),brightness);
            string newXml = s1+buf+s2;
            ofstream ofs;
            ofs.open("/etc/iqfiles/ps5268_default_default_tmp.xml");
            ofs<<newXml;
            ofs.close();
            plt_system("mv /etc/iqfiles/ps5268_default_default_tmp.xml /etc/iqfiles/ps5268_default_default.xml");
            bRet = true;
          }
    }
  }
  return bRet;
}

static int CAMERA_IMG_FMT=0;
static void imageCfgCallback(CameraHandle *cameraHandle, roller_eye::ImageConfig &config, uint32_t level) {
  PLOG_INFO(CORE_NODE_TAG,"CoreNode>>> Reconfigure: %d %d %f %d 0x%x, %d",
    config.image_width,
    config.image_height,
    (float)config.image_fps_den/config.image_fps_num,
    config.image_fmt,
    level, config.nightVisionSensitivity);
    g_cvStream_ptr->stopStream();
  uint32_t fmt=V4L2_PIX_FMT_YUV420;
  if(CAMERA_IMG_FMT==1){
    fmt=V4L2_PIX_FMT_YUYV;
  }
  if (0 ==config.cameraLight ){
    static bool bFirst = true;
    if (bFirst){
        string hwVer;
        PltConfig::getInstance()->getHwVersion(hwVer);
        if (hwVer.length() ==10 && hwVer[3] == '1' ){
          modifyBrightness(48);
        }else{
          modifyBrightness(32);
        }
        bFirst = false;
    }
  }else{
    static int prevCameraLight = 0;
    int brightness = (64*config.cameraLight+36)/100;
    PLOG_INFO(CORE_NODE_TAG,"%d %d\n", config.cameraLight, prevCameraLight);
    if (config.cameraLight != prevCameraLight){
      modifyBrightness(brightness);
    }
    prevCameraLight = config.cameraLight;
  }
  g_night_mode_ptr->setNightMode(config.image_night_mode, config.nightVisionSensitivity);
  cameraHandle->newCameraHandle(config.image_width, config.image_height,
             config.image_fps_num,config.image_fps_den, fmt, config.h264Quality, config.cameraLight);
}


bool stopDetect(stop_detectRequest & req,stop_detectResponse& res)
{
  switch(req.cmd){
    case 0:
    if (g_pile_detect_ptr){
      g_pile_detect_ptr->subDisconnectObj(0);
    }
    break;
    case 1:
    if (g_obj_detect_ptr){
      g_obj_detect_ptr->subDisconnectObj(0);
    }
    break;
    case 2:
    if (g_motion_detect_pub_ptr){
      g_motion_detect_pub_ptr->disconnectConnect();
    }
    break;
    default:
    break;
  }
   return true;
}

bool adjustExposureTime(adjust_exposure_timeRequest & req,adjust_exposure_timeResponse& res)
{
  float time = req.time;
  PLOG_INFO(CORE_NODE_TAG,"adjustExposureTime: %f",time);

  res.ret = -1;
   return res.ret;
}

int writeID(int id)
{
    key_t key;
    key = ftok(shmt_path, 1);
    if(key == -1){
        PLOG_INFO(CORE_NODE_TAG,"ftok failed");
    }
    int shmid;
    shmid = shmget(key,sizeof(recordData),FLAG);
    if(shmid == SHM_FAILED){
        PLOG_INFO(CORE_NODE_TAG,"share memory create fail!");
        return -1;
    }
    recordData *pRD;
    pRD = (recordData*)shmat(shmid,NULL,0);
    pRD->id = id;

    if(shmdt(pRD) !=0){
        PLOG_INFO(CORE_NODE_TAG,"share memory detach fail!");
        return -1;
    }
    return 0;
}

float readAngle(int id)
{
    float angle = 0.0;
    key_t key;
    key = ftok(shmt_path, 1);
    if(key == -1){
        cout << "ftok failed" << endl;
        PLOG_INFO(CORE_NODE_TAG,"ftok failed");
        return angle;
    }

    int shmid;
    recordData *pRD;
    shmid = shmget(key,sizeof(recordData),FLAG);
    pRD = (recordData*)shmat(shmid,NULL,0);
    if (pRD->id == pRD->rid){
      angle = pRD->angle;
    }

    if(shmdt(pRD) !=0){
        PLOG_INFO(CORE_NODE_TAG,"share memory detach fail!");
        return angle;
    }
    return angle;
}

/*************************************************
Function:       getDiffAngle
Description:  According to the feature points of the two pictures, the angle offset is calculated.
Input:          req- req.name is the file name of the picture
Output:      res- res.angle is the angle offset
Return:       true- success, false- fail
*************************************************/
bool getDiffAngle(getDiffAngleWhenPatrolRequest & req,getDiffAngleWhenPatrolResponse& res)
{
  float angle = 0.0;
  PLOG_INFO(CORE_NODE_TAG,"getDiffAngle: %s",req.name.c_str());
  cv::Mat src;

  if(g_cvStream_ptr->getCVImg(src)<0){
      res.angle = angle;
      return false;
  }

  cv::Mat grey;
  cv::resize(src,grey,cv::Size(FEATURE_DETECT_IMG_WIDTH,FEATURE_DETECT_IMG_HEIGHT));

  bool bRet =  cv::imwrite(  "/tmp/akaze.jpg",grey);
  if (bRet){
    FILE* stream = NULL;
    char buf[512]={0};
    size_t len = 0;
    //Call akaze to calculate the angle offset between them
    string strCmd = "/opt/ros/melodic/lib/roller_eye/akaze "+req.name+" /tmp/akaze.jpg";

    //Get angle offset from shared memory
    static int id = 0;
    writeID(id);
    system(strCmd.c_str());
    angle = readAngle(id);
    id++;
  }

  res.angle = angle;

  PLOG_INFO(CORE_NODE_TAG,"getDiffAngle angle: %f", res.angle);
  return bRet;
}

bool saveTmpPic(saveTmpPicForStartPathRequest & req,saveTmpPicForStartPathResponse& res)
{
    PLOG_INFO(CORE_NODE_TAG,"saveTmpPic: %s",req.name.c_str());
    cv::Mat src;
    if(g_cvStream_ptr->getCVImg(src)<0){
        return false;
    }

  cv::Mat grey;
  cv::resize(src,grey,cv::Size(FEATURE_DETECT_IMG_WIDTH,FEATURE_DETECT_IMG_HEIGHT));

  bool bRet =  cv::imwrite( req.name.c_str(),grey);
  if (bRet){
    string tmpFileName = req.name+"_tmp";
    rename(req.name.c_str(),tmpFileName.c_str());
  }
  PLOG_INFO(CORE_NODE_TAG,"saveTmpPic: %s End",req.name.c_str());
  return bRet;
}

void initGlobalData()
{
  DeviceDefaultConfig cfg;
  CAMERA_IMG_FMT=cfg.getCameraFormat();
  g_night_mode_ptr=make_shared<NightModeProcess>(cfg);
}
int main(int argc, char **argv)
{
  InitDZLog dzLog;
  ros::init(argc, argv, "CoreNode");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,CORE_NODE_DEBUG_LEVEL);
  ros::NodeHandle nh("~");
  ros::NodeHandle n("");
  auto r=make_shared<ros::NodeHandle>("~");

  initGlobalData();
  ros::ServiceServer adjustLightSrv = nh.advertiseService("adjust_light", &NightModeProcess::adjust, g_night_mode_ptr.get());
  ros::ServiceServer nightGetSrv = nh.advertiseService("night_get", &NightModeProcess::nightGet, g_night_mode_ptr.get());
  ros::ServiceServer stopDetectSrv = nh.advertiseService("stop_detect",stopDetect);
  ros::ServiceServer adjustExposureTimeSrv = nh.advertiseService("adjust_exposure_time",adjustExposureTime);
  ros::ServiceServer getDiffAngleSrv = nh.advertiseService("getDiffAngleWhenPatrol",getDiffAngle);
  ros::ServiceServer saveTmpPicSrv = nh.advertiseService("saveTmpPicForStartPath",saveTmpPic);

  CameraHandle* cameraHandle =CameraHandle::getInstance();
  cameraHandle->registVideoStreamEventCB(videoStreamEventCB);
  g_cvStream_ptr = std::make_shared<CVGreyImageStream>(ALIGN_IMG_WIDTH,ALIGN_IMG_HEIGHT);
  dynamic_reconfigure::Server<roller_eye::ImageConfig> cfgSrv(ros::NodeHandle(PARAM_VIDEO_PATH));
  cfgSrv.setCallback(boost::bind(&imageCfgCallback, cameraHandle, _1, _2));

  auto videoStreamPub = std::make_shared<VideoStreamPub>(nh, cameraHandle);
  auto audioStreamPub = std::make_shared<AudioStreamPub>(nh);
  auto picStreamPub = std::make_shared<PicStreamPub>(nh, cameraHandle);

  g_obj_detect_ptr = std::make_shared<ObjDetectPub>(nh, cameraHandle);
  g_obj_detect_ptr->setObjListener(std::bind(&VideoStreamPub::objCallback, videoStreamPub, placeholders::_1));

  g_pile_detect_ptr = std::make_shared<ObjDetectPub>(nh, cameraHandle,"chargingPile",true);
  g_pile_detect_ptr->setObjListener(std::bind(&VideoStreamPub::objCallback, videoStreamPub, placeholders::_1));

  DataPulisher<std_msgs::Int32,BackingUp> backup(r,"backing_up",10,make_shared<BackingUp>(nh),10);

  DataPulisher<sensor_msgs::Image,MonoImgPub> imgPub(r,"grey_img",100,make_shared<MonoImgPub>(),0);   //for calibrate

  g_motion_detect_ptr = std::make_shared<MotionDetect>();
  //DataPulisher<roller_eye::detect,MotionDetect> motionPub(r,"motion",100,g_motion_detect_ptr,0);
  g_motion_detect_pub_ptr = std::make_shared<DataPublisher2<roller_eye::detect,MotionDetect>>(r,"motion",100,g_motion_detect_ptr,1);

#if 0
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    uint32_t num_h264 = videoStreamPub->getNumSubscribers();
    uint32_t num_aac = audioStreamPub->getNumSubscribers();
    uint32_t num_jpg = picStreamPub->getNumSubscribers();
    if (num_aac + num_h264 + num_jpg == 0) {
      PLOG_INFO(CORE_NODE_TAG,"idle");
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
#else
  ros::spin();
#endif

  return 0;
}

