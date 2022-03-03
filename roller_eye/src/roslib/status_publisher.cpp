#include"roller_eye/status_publisher.h"

namespace roller_eye{
    StatusPublisher::StatusPublisher(ros::NodeHandle &n,bool pubLast,string name,int buff):
    mHandle(n),
    mPub(mHandle,name,buff,std::bind(&StatusPublisher::clinetIn,this,placeholders::_1)),
    mPubLast(pubLast)
    {
    }
    StatusPublisher::~StatusPublisher()
    {

    }
    void StatusPublisher::pubStatus(int* status,int len)
    {
        if(status==nullptr||len<=0){
            return;
        }
        vector<int32_t> data;
        for(int i=0;i<len;i++)
        {
            data.push_back(status[i]);
        }
        return pubStatus(data);
    }
    void StatusPublisher::pubStatus(vector<int32_t> &status)
    {
        lock_guard<mutex> lock(mMutex);
        if(mPubLast){
            mLast=status;
        }
        roller_eye::status msg;
        msg.status=status;
        mPub.publish(msg);
    }
    void StatusPublisher::clinetIn(int n)
    {
        lock_guard<mutex> lock(mMutex);
        if(mPubLast&&!mLast.empty()){
            roller_eye::status msg;
            msg.status=mLast;
            mPub.publish(msg);
        }
    }
}