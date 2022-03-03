#ifndef __ROS_TOOLS_H__
#define __ROS_TOOLS_H__
#include"ros/ros.h"
#include"std_msgs/Int32.h"

namespace roller_eye{

ros::Subscriber motor_disable(ros::NodeHandle &n);
void motor_enable(ros::Subscriber &sub);

ros::Subscriber motor_disable_adjust(ros::NodeHandle &n);
void motor_enable_adjust(ros::Subscriber &sub);

ros::Subscriber start_monitor_person(ros::NodeHandle &n);
void stop_monitor_person(ros::Subscriber &sub);

ros::Subscriber start_monitor_dog(ros::NodeHandle &n);
void stop_monitor_dog(ros::Subscriber &sub);

ros::Subscriber start_monitor_cat(ros::NodeHandle &n);
void stop_monitor_cat(ros::Subscriber &sub);

ros::Subscriber start_monitor_motion(ros::NodeHandle &n);
void stop_monitor_motion(ros::Subscriber &sub);

struct MontorHelper{
    ros::Subscriber person;
    ros::Subscriber cat;
    ros::Subscriber dog;
};

bool start_monitor_helper(ros::NodeHandle &n,struct MontorHelper& mon);
void stop_monitor_helper(struct MontorHelper& mon);
bool monitor_active_helper(struct MontorHelper& mon);

int dump_ros_node_param(const char* rosPath,const char* filePath);

class BackingUpHelper{
public:
    BackingUpHelper();
    void begin(ros::NodeHandle &n);
    bool isActive();
    void end();
private:
    void statusCallback(const std_msgs::Int32::ConstPtr& status);
    ros::Subscriber mBackingUp;
    bool mStartOk;
};
}
#endif