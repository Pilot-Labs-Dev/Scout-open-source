#include<list>
#include<mutex>
#include<memory>
#include<functional>
#include "ros/ros.h"
#include"roller_eye/rtmp_start.h"
#include"roller_eye/rtmp_stop.h"
#include "roller_eye/frame.h"
#include <dynamic_reconfigure/client.h>
#include <roller_eye/ImageConfig.h>
#include"roller_eye/plog_plus.h"
#include"roller_eye/block_queue.h"
#include"roller_eye/system_define.h"
#include "roller_eye/recorder_mp4.h"
#include "roller_eye/recorder_jpg.h"
#include"roller_eye/upload_cache_mgr.h"
#include "roller_eye/detect_record_get_status.h"

using namespace roller_eye;
using namespace std;

#define DETECT_RECORD_NODE  "DetectRecordNode"
static const string DETECTRECORD_NODE_NAME="DetectRecordNode";
static const string CACHE_ROOT_PATH_TMP="/userdata/roller_eye/cache_tmp/";
static const string CACHE_ROOT_PATH="/userdata/roller_eye/cache/";

const std::string REC_FILES_DIR = string(ROLLER_EYE_DINAMIC_FILE_HOME_PATH) + "media_files/";

class DetectRecordNode
{
  public:
  DetectRecordNode()
  :mCfgClient(PARAM_VIDEO_PATH, boost::bind(&DetectRecordNode::imageCfgCallback, this, _1))
  {
    m_recTimer = m_nh.createTimer(ros::Duration(60), &DetectRecordNode::vTimerCallback, this, false, false);
    startDiskCheck();
  }

    bool get_status(roller_eye::detect_record_get_statusRequest &req, roller_eye::detect_record_get_statusResponse &resp)
    {
        resp.status = m_recTimer.hasStarted();
        return true;
    }

  bool rtmp_start_callback(rtmp_startRequest&req,rtmp_startResponse&res)
  {
      if (m_recTimer.hasStarted()) {
        PLOG_DEBUG(REC_TAG, "It is recoding\n");
        return false;
      }

    PLOG_INFO(DETECT_RECORD_NODE,"start new rtmp session[%s/%s] live:%d,cache:%d,duration:%d\n",req.app.c_str(),req.name.c_str(),req.live,req.cache,req.duration);
    startRecord(req.app,req.name+".mp4",req.cache,req.live,req.duration*1000,req.flags);
     return true;
  }
  bool rtmp_stop_callback(rtmp_stopRequest&req,rtmp_stopResponse&res)
  {
    PLOG_INFO(DETECT_RECORD_NODE,"stop rtmp session[%s/%s]",req.app.c_str(),req.name.c_str());

    return true;
  }

  void imageCfgCallback(const roller_eye::ImageConfig& config)
  {
      if (config.image_width != mImgCfg.image_width ||
            config.image_height != mImgCfg.image_height ||
            config.image_fmt != mImgCfg.image_fmt ||
            config.image_fps_den != mImgCfg.image_fps_den ||
            config.image_fps_num != mImgCfg.image_fps_num ||
            config.h264Quality != mImgCfg.h264Quality ||
            config.cameraLight != mImgCfg.cameraLight){
          if (m_recTimer.hasStarted()){
            stopRecord();
          }
      }

      mImgCfg = config;
  }

  void record(const  std::string& app,const std::string&name,bool cache,bool live,int duration,uint32_t flags)
  {
     PLOG_INFO(DETECT_RECORD_NODE,"start record[%s/%s] live:%d,cache:%d,duration:%d\n",
                                 app.c_str(),name.c_str(),live,cache,duration);
     mDuration = duration;
     mStrName = name;
     m_recordmp4 = boost::make_shared<RecorderMP4>(CACHE_ROOT_PATH_TMP+name, mImgCfg.image_width,
                                       mImgCfg.image_height, (float)mImgCfg.image_fps_den/mImgCfg.image_fps_num);

     m_subH264 = m_nh.subscribe("CoreNode/h264", 100, &RecorderMP4::h264Callback, m_recordmp4);

      if (!m_subH264) {
          PLOG_ERROR(REC_TAG, "sub h264 failed.");
      }
      m_subAAC = m_nh.subscribe("CoreNode/aac", 100, &RecorderMP4::aacCallback, m_recordmp4);
      if (!m_subAAC) {
          PLOG_ERROR(REC_TAG, "sub aac failed.");
      }
      m_recordmp4->setDuration(duration/1000);
      m_recordmp4->startLoop();
      m_recTimer.setPeriod(ros::Duration(1));
      m_recTimer.start();
  }

  void startRecord(const std::string& app,const std::string&name,bool cache,bool live,int duration,uint32_t flags)
  {
    if (duration<=0){
      return;
    }

      auto th=std::thread([this,app,name,cache,live,duration,flags](){
              record(app,name,cache,live,duration,flags);
      });
      th.detach();
  }

  void stopRecord()
  {
    m_subH264.shutdown();
    m_subAAC.shutdown();
    m_recTimer.stop();
    if (m_recordmp4) {
        m_recordmp4->stopLoop();
        string opath=CACHE_ROOT_PATH_TMP+mStrName;
        string npath=CACHE_ROOT_PATH+mStrName;
        rename(opath.c_str(),npath.c_str());
    }
  }

  void vTimerCallback(const ros::TimerEvent& event)
  {
      if (m_recordmp4->isCompleted()){
        stopRecord();
      }
  }

  void startDiskCheck()
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

  private:
    int mDuration;
    string mStrName;
    ros::Timer m_recTimer;
    ros::NodeHandle m_nh;
    ros::Subscriber m_subH264;
    ros::Subscriber m_subAAC;
    ros::Subscriber m_subJPG;
    ros::Subscriber m_subThumb;
    ros::ServiceServer m_statusSrv;
    roller_eye::ImageConfig mImgCfg;
    boost::shared_ptr<RecorderMP4> m_recordmp4;
    dynamic_reconfigure::Client<roller_eye::ImageConfig> mCfgClient;
};

 int main(int argc, char **argv)
{
  ros::init(argc, argv, DETECTRECORD_NODE_NAME);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,RTMP_NODE_DEBUG_LEVEL);
  ros::NodeHandle nh("~");
  DetectRecordNode drNode;
  auto start=nh.advertiseService("rtmp_start", &DetectRecordNode::rtmp_start_callback, &drNode);
  auto stop=nh.advertiseService("rtmp_stop", &DetectRecordNode::rtmp_stop_callback, &drNode);
  auto status=nh.advertiseService("get_status", &DetectRecordNode::get_status, &drNode);
  PLOG_INFO(DETECT_RECORD_NODE,"start detect record node ok!\n");
  ros::spin();
  PLOG_INFO(DETECT_RECORD_NODE,"detect record node exit!\n");
  return 0;
}

