#include<stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include"roller_eye/ros_tools.h"
#include"std_msgs/String.h"
#include"roller_eye/status.h"
#include"roller_eye/system_define.h"
#include"roller_eye/plt_tools.h"

namespace roller_eye{
static void   string_callback(const std_msgs::String::ConstPtr& msg)
{
    //do nothing
    //ROS_DEBUG("%s\n",msg->data.c_str());
}
ros::Subscriber motor_disable(ros::NodeHandle &n)
{
    return n.subscribe("MotorNode/disable",2,string_callback);
}
void motor_enable(ros::Subscriber &sub)
{
    sub.shutdown();
}
ros::Subscriber motor_disable_adjust(ros::NodeHandle &n)
{
    return n.subscribe("MotorNode/disable_adjust",2,string_callback);
}
void motor_enable_adjust(ros::Subscriber &sub)
{
    sub.shutdown();
}

ros::Subscriber start_monitor_person(ros::NodeHandle &n)
{
    return n.subscribe("CloudNode/person_monitor",2,string_callback);
}
void stop_monitor_person(ros::Subscriber &sub)
{
    sub.shutdown();
}
ros::Subscriber start_monitor_dog(ros::NodeHandle &n)
{
    return n.subscribe("CloudNode/dog_monitor",2,string_callback);
}
void stop_monitor_dog(ros::Subscriber &sub)
{
    sub.shutdown();
}
ros::Subscriber start_monitor_cat(ros::NodeHandle &n)
{
    return n.subscribe("CloudNode/cat_monitor",2,string_callback);
}
void stop_monitor_cat(ros::Subscriber &sub)
{
    sub.shutdown();
}
ros::Subscriber start_monitor_motion(ros::NodeHandle &n)
{
    return n.subscribe("CloudNode/motion_monitor",2,string_callback);
}
void stop_monitor_motion(ros::Subscriber &sub)
{
    sub.shutdown();
}
bool start_monitor_helper(ros::NodeHandle &n,struct MontorHelper& mon)
{
    mon.person=start_monitor_person(n);
    mon.cat=start_monitor_cat(n);
    mon.dog=start_monitor_dog(n);
    if(monitor_active_helper(mon)){
        return true;
    }else{
        stop_monitor_helper(mon);
        return false;
    }
}
void stop_monitor_helper(struct MontorHelper& mon)
{
    if (mon.person){
        stop_monitor_person(mon.person);
    }
    if (mon.cat){
        stop_monitor_cat(mon.cat);
    }
    if (mon.dog){
        stop_monitor_dog(mon.dog);
    }
}
bool monitor_active_helper(struct MontorHelper& mon)
{
    return (mon.person && mon.cat && mon.dog);
}
int dump_ros_node_param(const char* rosPath,const char* filePath)
{
    char cmd[256];

    if(snprintf(cmd,sizeof(cmd),"rosparam dump %s %s",filePath,rosPath)>=((int)sizeof(cmd)-1)){
        ROS_ERROR("cmd lenght over flow");
        return -1;
    }
    int ret;
    if((ret=plt_system(cmd))<0){
        ROS_ERROR("[%s] fail",cmd);
    }
    return ret;
}
    BackingUpHelper::BackingUpHelper()
    {

    }
    void BackingUpHelper::begin(ros::NodeHandle &n)
    {
        ROS_ERROR("BackingUpHelper::begin");
        if(!mBackingUp){
            ROS_ERROR("BackingUpHelper::begin subscribe backing_up");
            mStartOk=false;
            mBackingUp=n.subscribe("CoreNode/backing_up",100,&BackingUpHelper::statusCallback,this);
        }
    }
    void BackingUpHelper::end()
    {
        if(mBackingUp){
            mBackingUp.shutdown();
        }
    }
    bool BackingUpHelper::isActive()
    {
        return mBackingUp;
    }
    void   BackingUpHelper::statusCallback(const std_msgs::Int32::ConstPtr& status)
    {
        if(!mStartOk){
            if(status->data!=roller_eye::status::BACK_UP_DETECT){
                return;
            }
            mStartOk=true;
        }
        switch(status->data){
            case roller_eye::status::BACK_UP_SUCCESS:
            case roller_eye::status::BACK_UP_FAIL:
            case roller_eye::status::BACK_UP_INACTIVE:
                // ROS_INFO("backup exit status=%d",status->data);
                if(mBackingUp){
                    mBackingUp.shutdown();
                }
            default:
                break;
        }
    }
}