
#pragma once

#include<atomic>
#include"plt_assert.h"
#include<functional>
#include"ros/ros.h"

using namespace std;
namespace roller_eye{

typedef std::function<void(void)> ClientIn2;
typedef std::function<void(void)> ClientOut2;

template<class T>
 class Publisher2{
public:
    Publisher2(ros::NodeHandle &n,string name,int buff,const ClientIn2& in,const ClientOut2& out):
    mConnec(0),
    mIn(in),
    mOut(out)
    {
        init(n,name,buff);
    }
    Publisher2(ros::NodeHandle &n,string name,int buff,const ClientIn2& in):
    mConnec(0),
    mIn(in),
    mOut(nullptr)
    {
        init(n,name,buff);
    }
    Publisher2(ros::NodeHandle &n,string name,int buff):
    mConnec(0),
    mIn(nullptr),
    mOut(nullptr)
    {
        init(n,name,buff);
    }
    ~Publisher2()
    {
    }

    void subcriberConnect(const ros::SingleSubscriberPublisher& sub)
    {  
        if(mIn){
            mIn();   
        }
    }
    void disconnectConnect(const ros::SingleSubscriberPublisher& sub)
    {
        if(mOut){
            mOut();   
        }
    }
    void unSubConnect()
    {
        if(mOut){
            mOut();   
        }
    }
    void publish(const T& msg){
        mPub.publish(msg);
    }

    string getName(){
        return mName;
    }
protected:
     void init(ros::NodeHandle &n,string name,int buff)
     {
         mName = name;
         const ros::SubscriberStatusCallback conCB=bind(&Publisher2::subcriberConnect,this,placeholders::_1);
         const ros::SubscriberStatusCallback disCB=bind(&Publisher2::disconnectConnect,this,placeholders::_1);
         mPub=n.advertise<T>(name,buff,conCB,disCB);
     }
     string mName;
    ros::Publisher mPub;
    atomic<int> mConnec;
    ClientIn2 mIn;
    ClientOut2 mOut;
};
}