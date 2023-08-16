#include "ros/ros.h"
#include "roller_eye/algo_utils.h"
#include "roller_eye/algo_roll.h"
#include "roller_eye/algo_move.h"
#include "roller_eye/algo_action.h"
#include "roller_eye/ai_get_detect_setting.h"
#include "roller_eye/ai_set_detect_setting.h"

#include "roller_eye/system_define.h"
#include "roller_eye/plt_config.h"
#include "roller_eye/param_utils.h"
#include "nlohmann/json.hpp"


using namespace roller_eye;
using namespace std;

#define UTIL_NODE_TAG    "UtilNode"

static unique_ptr<AlgoUtils> algoUtils;


//param process
class ParamGroupProcessHandle{
public:
    ParamGroupProcessHandle(const string& groupKey):
    mGroupKey(groupKey),
    mConfigPath(ROLLER_EYE_CONFIG_BASE+mGroupKey)
    {
    }
    virtual ~ParamGroupProcessHandle()
    {

    }
    virtual int setParam(json& param)=0;
    virtual int getParam(json& param)=0;
    const string& getGroupKey()
    {
        return mGroupKey;
    }
protected:
    string mGroupKey;
    string mConfigPath;
    ros::NodeHandle mHandle;
};

class ParamGroupJsonProcessHandle:public ParamGroupProcessHandle{
public:
    ParamGroupJsonProcessHandle(const string& groupKey):
    ParamGroupProcessHandle(groupKey)
    {
        mConfig=PltConfig::getInstance();
    }
    int setParam(json& param)
    {
        json data;
        if(extract(param,data)<0){
            ROS_ERROR("extract [%s] param fail",mGroupKey.c_str());
            return -1;
        }
        mConfig->setMonitorParam(data);
        return update();
    }
    int getParam(json& param)
    {
        pack(param);
        return 0;
    }
    const string& getGroupKey()
    {
        return mGroupKey;
    }
protected:
    virtual int update()=0;
    virtual int extract(json& param,json& data)=0;
    virtual void pack(json& data)=0;

    PltConfig *mConfig;
};
class ParamMonitorProcessHandle:public ParamGroupJsonProcessHandle{
public:
    ParamMonitorProcessHandle():
    ParamGroupJsonProcessHandle(PARAM_SYSTEM_MONITOR_GROUP),
    mCharging(false)
    {
        load_monitor_param(mParam);
        mBatteryStatus=mHandle.subscribe("SensorNode/simple_battery_status",10,&ParamMonitorProcessHandle::batteryStatusCallback,this);
    }
private:
    int update()
    {
        int ret=mMonitor.setupZone(mParam);
        if(ret<0){
            ROS_ERROR("set zone fail");
        }
        lock_guard<mutex> lock(mMutex);
        mMonitor.setupMonitor(mParam,!mCharging);
        return ret;
    }
    void pack(json& param)
    {
        dump_monitor_param(mParam,param);
    }
    int extract(json& param,json& data)
    {
        MonitorParam mon;
        if(parse_monitor_param(param,mon)<0){
            return -1;
        }
        lock_guard<mutex> lock(mMutex);
        mParam=std::move(mon);
        pack(data);
        return 0;
    }
    void batteryStatusCallback(const statusConstPtr &s)
    {
        lock_guard<mutex> lock(mMutex);
        bool charging=s->status[2];
        /*if(charging!=mCharging)*/{
            mCharging=charging;
            mMonitor.setupMonitor(mParam,!mCharging);
        }
    }
    ros::NodeHandle mHandle;
    MonitorParam mParam;
    MonitorHandle mMonitor;
    ros::Subscriber mBatteryStatus;
    int mCharging;
    mutex mMutex;
};


static bool algoRoll(roller_eye::algo_rollRequest &req, roller_eye::algo_rollResponse &resp){
  PLOG_DEBUG(UTIL_NODE_TAG, "algoRollEx, angle:%.2frad speed:%.2frad/s timeout:%dms error:%.3f", req.angle, req.rotatedSpeed, req.timeout, req.error);
  resp.ret = algoUtils->rollEx(req.angle, req.rotatedSpeed, req.timeout, req.error);
  PLOG_DEBUG(UTIL_NODE_TAG, "algoRollEx ret:%d", resp.ret);
  return true;
}

static bool algoMove(roller_eye::algo_moveRequest &req, roller_eye::algo_moveResponse &resp){
  PLOG_DEBUG(UTIL_NODE_TAG, "algoMove, xDistance:%.2fm yDistance:%.2fm speed:%.2fm/s", req.xDistance, req.yDistance, req.speed);

  //resp.ret = algoUtils->move(req.xDistance, req.yDistance, req.speed);
  resp.ret = algoUtils->moveEx(req.xDistance, req.yDistance, req.speed);
  PLOG_DEBUG(UTIL_NODE_TAG, "algoMove ret:%d", resp.ret);
  return true;
}

static bool algoAction(roller_eye::algo_actionRequest &req, roller_eye::algo_actionResponse &resp){
  PLOG_DEBUG(UTIL_NODE_TAG, "algoMove, xSpeed:%.2fm/s ySpeed:%.2fm/s time:%dms rotatedSpeed:%.2frad/s", req.xSpeed, req.ySpeed, req.time, req.rotatedSpeed);

  resp.ret = algoUtils->action(req.xSpeed, req.ySpeed, req.rotatedSpeed, req.time);
  PLOG_DEBUG(UTIL_NODE_TAG, "algoAction ret:%d", resp.ret);
  return true;
}

static bool checkAiSettingDataIsOk(json& setting, json& refinedSetting){
  MonitorParam targetParam;
  MonitorParam tempParam;

  if(parse_monitor_param(setting, tempParam)<0){
    return false;
  }

  targetParam = std::move(tempParam);

  dump_monitor_param(targetParam, refinedSetting);


  return true;
}

static bool aiGetDetectSetting(roller_eye::ai_get_detect_settingRequest &req, roller_eye::ai_get_detect_settingResponse &resp){
  PltConfig *config = PltConfig::getInstance();

  json settingJson;
  config->getMonitorParam(settingJson);

  resp.setting = settingJson.dump();

  return true;
}

static bool aiSetDetectDetting(roller_eye::ai_set_detect_settingRequest &req, roller_eye::ai_set_detect_settingResponse &resp){

  try{
    json settingJson = json::parse(req.setting);

    auto handle = make_shared<ParamMonitorProcessHandle>();

    if(handle->setParam(settingJson) < 0){
      resp.ret = -1;
    }
  }
  catch(const std::exception& e){
    resp.ret = -1;
  }

  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "UtilNode");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, UTIL_NODE_DEBUG_LEVEL);
  auto privateNh = make_shared<ros::NodeHandle>("~");

  algoUtils = unique_ptr<AlgoUtils>(new AlgoUtils());

  ros::ServiceServer algoRollSrv = privateNh->advertiseService("algo_roll", algoRoll);
  ros::ServiceServer algoMoveSrv = privateNh->advertiseService("algo_move", algoMove);
  ros::ServiceServer algoActionSrv = privateNh->advertiseService("algo_action", algoAction);

  ros::ServiceServer aiDetectEnableOrDisableSrv = privateNh->advertiseService("ai_set_detect_setting", aiSetDetectDetting);
  ros::ServiceServer aiDetectSettingGetSrv = privateNh->advertiseService("ai_get_detect_setting", aiGetDetectSetting);

  ros::spin();

  return 0;
}