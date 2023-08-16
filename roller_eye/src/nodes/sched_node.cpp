#include<ctime>
#include<map>
#include"ros/ros.h"
#include"roller_eye/timer_task_sched.h"
#include"roller_eye/system_define.h"
#include"roller_eye/sched_add_mod.h"
#include"roller_eye/sched_delete.h"
#include"roller_eye/sched_list.h"
#include"roller_eye/task.h"
#include"roller_eye/plt_tools.h"
#include"roller_eye/nav_patrol.h"
#include"nlohmann/json.hpp"
#include"roller_eye/status.h"
#include"roller_eye/ros_tools.h"
#include"roller_eye/util_class.h"
#include "roller_eye/record.h"
#include "roller_eye/record_start.h"
#include "roller_eye/record_stop.h"

using namespace std;
using namespace roller_eye;

static void setTaskMsg(task& tsk,TimerTaskPtr& ptr)
{
    tsk.id=ptr->id;
    tsk.name=ptr->name;
    tsk.type=ptr->type;
    tsk.repeateType=ptr->repeatType;
    for(auto day:ptr->days){
        tsk.repeateDays.push_back(day);
    }
    tsk.timeYear=ptr->startTime.tm_year+1900;
    tsk.timeMonth=ptr->startTime.tm_mon+1;
    tsk.timeDay=ptr->startTime.tm_mday;
    tsk.timeHour=ptr->startTime.tm_hour;
    tsk.timeMinute=ptr->startTime.tm_min;
    tsk.timeSecond=ptr->startTime.tm_sec;
    tsk.expire=ptr->expire;
    tsk.param=ptr->param;
    tsk.notice=ptr->notice;
    tsk.active=ptr->active;
}

class TimerTaskHandler{
public:
    TimerTaskHandler():
    mLocal("~")
    {
        mTask=mLocal.advertise<roller_eye::task>("timer_task",100);
    }
    virtual ~TimerTaskHandler()
    {

    }
    void excuteTask(TimerTaskPtr &taskptr)
    {
        try{
            roller_eye::task tsk;
            setTaskMsg(tsk,taskptr);
            mTask.publish(tsk);
            nlohmann::json param=nlohmann::json::parse(taskptr->param);
            doTask(taskptr,param);
        }catch(std::exception &e){
            ROS_WARN("task param[%s],what:%s",taskptr->param.c_str(),e.what());
        }
    }
protected:
    virtual void doTask(TimerTaskPtr &task,nlohmann::json& param)=0;
    ros::NodeHandle mGlobal;
    ros::NodeHandle mLocal;
    ros::Publisher mTask;
};
class PatrolTaskHandler:public TimerTaskHandler{
public:
    PatrolTaskHandler():
    mNeedMonitor(0),
    mHasPatrolTask(false),
    mWaitingPatral(false),
    mRecording(false)
    {
        const string RecordNodeName="RecorderAgentNode";
        mClient                = mGlobal.serviceClient<nav_patrol>("NavPathNode/nav_patrol");
        mNavStatus       = mGlobal.subscribe("NavPathNode/status",5,&PatrolTaskHandler::navStatusCallback,this);
        mRecordStartClient = mGlobal.serviceClient<record_start>(RecordNodeName+"/record_start");
        mRecordStopClient = mGlobal.serviceClient<record_stop>(RecordNodeName+"/record_stop");
        mBattery=mGlobal.subscribe("SensorNode/simple_battery_status",5,&PatrolTaskHandler::batteryStatusCallback,this);
    }
private:
    void doTask(TimerTaskPtr &task,nlohmann::json& param)
    {
        if (!mChargingStatus){   //don't patrol when scout not charging
            return;
        }
        nav_patrol patrol;
        patrol.request.name=param["route"];
        patrol.request.isFromOutStart = mChargingStatus;
        ROS_DEBUG("start patrol[%s]",patrol.request.name.c_str());
        if(!mClient.call(patrol)){
            ROS_ERROR("start patrol error");
            mNeedMonitor=0;
            mWaitingPatral=false;
            mHasPatrolTask=false;
            mRecording         = false;
        }else{
            mNeedMonitor=param["record"];
            mWaitingPatral=true;
            mHasPatrolTask=true;
        }
    }
    void navStatusCallback(const statusConstPtr &status)
    {
        ROS_DEBUG("nav status=%d",status->status[1]);
        if(mWaitingPatral&&!status->status[1]){
            ROS_DEBUG("waiting patrol active");
            return;
        }
        mWaitingPatral=false;
        if(status->status[1]){
            if(mHasPatrolTask && mNeedMonitor){
                record_start::Request req;
                record_start::Response res;

                if (!mRecording){
                    // ROS_INFO("record start command\n");
                    //req.type= record::RECORD_TYPE_RECORD;
                    req.type= record::RECORD_TYPE_SCHED_RECORD;
                    req.mode= 1;
                    req.duration= 30000;
                    req.count= 0;                   //keep on recording until stop patrol

                    // ROS_INFO("record start call\n");
                    if(mRecordStartClient.call(req,res)){
                        if(res.status==status::PROCESS_OK){
                            // ROS_INFO("record start call suc\n");
                            mRecording = true;
                        }
                    }
                }
            }
        }else{
            if (mRecording){
                record_stop::Request req;
                record_stop::Response res;
                mRecording = false;
                req.type= record::RECORD_TYPE_SCHED_RECORD;
                // ROS_INFO("record stop call\n");
                if(mRecordStopClient.call(req,res)){
                    if(res.status==status::PROCESS_OK){
                        ROS_INFO("record stop call suc\n");
                    }
                }
            }
        }

        if(mHasPatrolTask && !status->status[1]){//patrol complete
            mHasPatrolTask=false;
        }
    }

    void batteryStatusCallback(const statusConstPtr &s)
    {
        mChargingStatus=s->status[2];
    }

    ros:: ServiceClient mClient;
    ros:: ServiceClient mRecordStartClient;
    ros:: ServiceClient mRecordStopClient;
    ros::Subscriber mNavStatus;
    MontorHelper mMonitor;
    int mNeedMonitor;
    bool mHasPatrolTask;
    bool mWaitingPatral;
    bool mRecording;
    int mChargingStatus;
    ros::Subscriber mBattery;
    BackingUpHelper mBackingUp;
};
class TimerTaskMgr{
public:
    TimerTaskMgr():
    mLocal("~"),
    mNextTask(nullptr)
    {
        mTimer=mLocal.createTimer(ros::Duration(1.0),&TimerTaskMgr::timerCallback,this);
        mAdd=mLocal.advertiseService("sched_add_mod",&TimerTaskMgr::addModTask,this);
        mDel=mLocal.advertiseService("sched_delete",&TimerTaskMgr::delTask,this);
        mList=mLocal.advertiseService("sched_list",&TimerTaskMgr::listTask,this);
        mTaskHandler[TIMER_TASK_TYPE_PATROL]=make_shared<PatrolTaskHandler>();
    }
private:
    void updateNextTask()
    {
        int delay;
        mNextTask=mSched.getNextTask(delay);
        if(mNextTask){
            // ROS_INFO("next task delay:%d",delay);
            if(delay>0){
                mTimer.setPeriod(ros::Duration(delay));
            }else{
                mTimer.setPeriod(ros::Duration(0.1));
            }
        }else{
            // ROS_INFO("today no timer task,next day delay=%d",delay);
            mTimer.setPeriod(ros::Duration(delay));
        }
    }
    void timerCallback(const ros::TimerEvent& evt)
    {
        ROS_DEBUG("schedule timer");
        if(!checkTimeSynced(0)){
            ROS_DEBUG("system time not synced,wait");
            return;
        }
        if(mNextTask){
          if(!detectProcessIsExited("python /userdata/roller_eye/scratch/scripts/")){
            mSched.schedTask(mNextTask);
            // ROS_INFO("task[%d] schedule",mNextTask->id);

            executeTask(mNextTask);
          }
        }
        updateNextTask();
    }
    bool addModTask(sched_add_modRequest& req,sched_add_modResponse& res)
    {
        bool add=false;
        auto task=mSched.findTask(req.tsk.id);
        if(!task){
            task=make_shared<TimerTask>();
            add=true;
        }
        task->name=req.tsk.name;
        task->type=req.tsk.type;
        task->repeatType=req.tsk.repeateType;
        task->days.clear();
        for(auto day:req.tsk.repeateDays){
            task->days.push_back(day);
        }
        char buff[64];
        snprintf(buff,sizeof(buff),"%d-%d-%d %d:%d:%d",req.tsk.timeYear,req.tsk.timeMonth,req.tsk.timeDay,req.tsk.timeHour,req.tsk.timeMinute,req.tsk.timeSecond);
        if(strptime(buff,"%Y-%m-%d %H:%M:%S",&task->startTime)==NULL){
            ROS_ERROR("bad time:%s",buff);
            return false;
        }
        task->expire=req.tsk.expire;
        task->param=req.tsk.param;
        task->notice=req.tsk.notice;
        task->active=req.tsk.active;
        ROS_DEBUG("request add/mod task[%s],start time:%s]",task->name.c_str(),buff);
        bool ret=add?(mSched.addTask(task)==0):(mSched.modTask(task)==0);
        if(ret){
            ROS_DEBUG("add/mod task ok,id=%d",task->id);
            updateNextTask();
            res.id=task->id;
        }
        return ret;
    }
    bool delTask(sched_deleteRequest& req,sched_deleteResponse& res)
    {
        bool ret=true;
        for(auto id:req.ids){
            if(mSched.delTask(id)!=0){
                ret=false;
            }
        }
        updateNextTask();
        return ret;
    }

    bool listTask(sched_listRequest& req,sched_listResponse& res)
    {
        auto tsks=mSched.getTasks(req.type,req.startID,req.count);
        for(auto&ptr:tsks){
            task tsk;
            setTaskMsg(tsk,ptr);
            res.result.push_back(tsk);
        }
        return true;
    }
    void executeTask(TimerTaskPtr& task)
    {
        auto handler=mTaskHandler.find(task->type);
        if(handler==mTaskHandler.end()){
            ROS_WARN("no hander to execute task type[%s]",task->type.c_str());
            return;
        }
        handler->second->excuteTask(task);
    }
    ros::NodeHandle mLocal;
    ros::Timer mTimer;
    ros::ServiceServer mAdd;
    ros::ServiceServer mDel;
    ros::ServiceServer mList;
    TimerTaskScheduler mSched;
    TimerTaskPtr mNextTask;
    map<string,shared_ptr<TimerTaskHandler>> mTaskHandler;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SchedNode");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,SCHED_NODE_DEBUG_LEVEL);
  TimerTaskMgr mgr;
  ros::spin();
  return 0;
}