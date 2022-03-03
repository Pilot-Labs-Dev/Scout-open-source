#ifndef __ROS__LIB_STATUS_PUBLISHER__
#define __ROS__LIB_STATUS_PUBLISHER__
#include"ros/ros.h"
#include<vector>
#include<mutex>
#include"roller_eye/status.h"
#include"roller_eye/publisher.h"

using namespace std;
namespace roller_eye{
class StatusPublisher{
public:
    StatusPublisher(ros::NodeHandle &n,bool pubLast=true,string name="status",int buff=100);
    ~StatusPublisher();

    void pubStatus(vector<int32_t> &status);
    void pubStatus(int* status,int len);

    void clinetIn(int n);

protected:
    ros::NodeHandle mHandle;
    roller_eye::Publisher<roller_eye::status> mPub;
    vector<int32_t> mLast;
    bool mPubLast;
    mutex mMutex;
};
}
#endif