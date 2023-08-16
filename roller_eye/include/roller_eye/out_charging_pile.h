
#include "ros/ros.h"
#include<geometry_msgs/Twist.h>
#include"std_msgs/Int32.h"
#include "roller_eye/sound_effects_mgr.h"
#include "roller_eye/motor.h"

namespace roller_eye
{
class OutChargingPile{
public:
    OutChargingPile(shared_ptr<ros::NodeHandle> &node):
    mNodeHandle(node)
    {
        mSysEvtPub = mNodeHandle->advertise<std_msgs::Int32>("/system_event",1);
        mPub=mNodeHandle->advertise<geometry_msgs::Twist>("/cmd_vel",50);
        mBatteryStatus=mNodeHandle->subscribe("/SensorNode/simple_battery_status",10,
                                                                                      &OutChargingPile::batteryStatusCallback,this);
    }
    int doOut()
    {
        if (!mCharging){
            return -1;
        }
        if (mOuting){
            return 0;
        }

        std_msgs::Int32 event;
        event.data = SYSEVT_OUTPILE;
        mSysEvtPub.publish(event);

        mOuting=true;
        auto th=std::thread([this](){
            geometry_msgs::Twist vel;
            vel.linear.y=0.5*MACC_MAX_SPEED_Y;
            mPub.publish(vel);
            usleep(100*1000);
            mOuting=false;
        });
        th.detach();
        return 0;
    }
private:
    void batteryStatusCallback(const statusConstPtr &s)
    {
        mCharging=s->status[2];
    }
private:
    bool mOuting=false;
    bool mCharging=false;
    ros::Publisher mPub;
    shared_ptr<ros::NodeHandle> mNodeHandle;
    ros::Subscriber mBatteryStatus;
    ros::Publisher mSysEvtPub;
};
}