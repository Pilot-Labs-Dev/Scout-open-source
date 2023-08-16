#include<list>
#include<mutex>
#include<memory>
#include<functional>
#include "ros/ros.h"
#include"roller_eye/rtmp_start.h"
#include"roller_eye/rtmp_stop.h"
#include "roller_eye/frame.h"
#include "roller_eye/rtmp_pub.h"
#include"roller_eye/plog_plus.h"
#include"roller_eye/block_queue.h"
#include"roller_eye/system_define.h"

using namespace roller_eye;
using namespace std;

#define RTMP_STREAM_NODE  "RTMPNode"
static const string RTMP_NODE_NAME="RTMPNode";

class RTMPSessions{
public:
  RTMPSessions():mHandle("")
  {
    mRtmpStop=mHandle.serviceClient<rtmp_stop>(RTMP_NODE_NAME+"/rtmp_stop");
  }
  ~RTMPSessions()
  {
    PLOG_DEBUG(RTMP_STREAM_NODE,"RTMPSessions destruct");
    closeliveStream();
  }
  int removeSession(std::string& app,std::string& name)
  {
    int ret=0;

    for(auto it=mSessions.begin();it!=mSessions.end();){
      if(it->compareApp(app,name)){
        ret++;
        it->stopLoop();
        mSessions.erase(it++);
      }else{
        ++it;
      }
    }
    if(ret>0&&liveSessionCount()==0){
      closeliveStream();
    }
    return ret;
  }
  int addSession(std::string& app,std::string&name,bool cache,bool live,int duration,uint32_t flags)
  {
    for(auto& s:mSessions){
      if(s.compareApp(app,name)){
        PLOG_ERROR(RTMP_STREAM_NODE,"same rtmp application");
        return -1;
      }
    }
    if(live){
      openLiveStream();
    }
    mSessions.emplace_back(app,name,cache,live,duration,flags);
    mSessions.rbegin()->setEndCallback(bind(&RTMPSessions::sessionStopCallback,this,placeholders::_1,placeholders::_2,placeholders::_3));
    mSessions.rbegin()->startLoop();
    return mSessions.rbegin()->isNetworkOK()?1:0;
  }
  void h264Callback(frameConstPtr frame)
  {
    for(auto& session:mSessions){
      if(session.isLive()){
        session.h264Callback(frame);
      }
    }
  }
  void aacCallback(frameConstPtr frame)
  {
    for(auto& session:mSessions){
      if(session.isLive()){
        session.aacCallback(frame);
      }
    }
  }
  void sessionStopCallback(string& app,string& name,bool error)
  {
    PLOG_INFO(RTMP_STREAM_NODE,"session[%s/%s] stop,error:%d\n",app.c_str(),name.c_str(),error);
    //do not call "removeSession(app,name,!error)" directly,RTMPSessions is thread unsafe
    rtmp_stop stop;
    stop.request.app=app;
    stop.request.name=name;
    if(!error){
      if(CacheMgr::getInstance()->setReady(name)!=0){
        PLOG_ERROR(RTMP_STREAM_NODE,"set [%s/%s] ready fail!",app.c_str(),name.c_str());
      }
    }
    if(!mRtmpStop.call(stop)){
      PLOG_ERROR(RTMP_STREAM_NODE,"call remove function fail");
    }
    PLOG_DEBUG(RTMP_STREAM_NODE,"call remove function ok,ret=%d",stop.response.result);
  }
private:
  void openLiveStream()
  {
    if(!mAudio||!mVideo){
      if(!mVideo){
          PLOG_INFO(RTMP_STREAM_NODE,"rtmp sub h264");
          mVideo=mHandle.subscribe("CoreNode/h264", 1000, &RTMPSessions::h264Callback, this);
        }
        if(!mAudio){
          PLOG_INFO(RTMP_STREAM_NODE,"rtmp sub aac");
          mAudio=mHandle.subscribe("CoreNode/aac", 1000, &RTMPSessions::aacCallback, this);
        }
        if(!mVideo||!mAudio){
          PLOG_ERROR(RTMP_STREAM_NODE,"Viedo/Audio not ready!!!");
        }
    }
  }
  void closeliveStream()
  {
    if(mVideo){
        mVideo.shutdown();
        PLOG_INFO(RTMP_STREAM_NODE,"shut down h264");
      }
      if(mAudio){
        mAudio.shutdown();
        PLOG_INFO(RTMP_STREAM_NODE,"shut down aac");
      }
  }
  int liveSessionCount()
  {
    int liveCnt=0;
    for(auto& s:mSessions){
      if(s.isLive()){
        liveCnt++;
      }
    }
    return liveCnt;
  }

  list<RtmpPub> mSessions;
  ros::NodeHandle mHandle;
  ros::Subscriber mVideo;
  ros::Subscriber mAudio;
  ros::ServiceClient mRtmpStop;
};

static shared_ptr<RTMPSessions> g_Sessions;
static bool rtmp_start_callback(rtmp_startRequest&req,rtmp_startResponse&res)
{
  PLOG_INFO(RTMP_STREAM_NODE,"start new rtmp session[%s/%s] live:%d,cache:%d,duration:%d\n",req.app.c_str(),req.name.c_str(),req.live,req.cache,req.duration);
  res.result=g_Sessions->addSession(req.app,req.name,req.cache,req.live,req.duration*1000,req.flags);
  return true;
}
static bool rtmp_stop_callback(rtmp_stopRequest&req,rtmp_stopResponse&res)
{
  PLOG_INFO(RTMP_STREAM_NODE,"stop rtmp session[%s/%s]",req.app.c_str(),req.name.c_str());
  res.result=g_Sessions->removeSession(req.app,req.name);
  return true;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, RTMP_NODE_NAME);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,RTMP_NODE_DEBUG_LEVEL);
  ros::NodeHandle nh("~");
  g_Sessions=make_shared<RTMPSessions>();
  auto start=nh.advertiseService("rtmp_start",rtmp_start_callback);
  auto stop=nh.advertiseService("rtmp_stop",rtmp_stop_callback);
  PLOG_INFO(RTMP_STREAM_NODE,"start rtmp node ok!\n");
  ros::spin();
  PLOG_INFO(RTMP_STREAM_NODE,"rtmp node exit!\n");
  return 0;
}

