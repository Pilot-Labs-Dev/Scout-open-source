
#ifndef __ROLLER_EYE_PUBLISHER__H__
#define __ROLLER_EYE_PUBLISHER__H__
#include<atomic>
#include"plt_assert.h"
#include<functional>
#include"ros/ros.h"

using namespace std;
namespace roller_eye{

typedef std::function<void(int)> ClientIn;
typedef std::function<void(int)> ClientOut;

template<class T>
 class Publisher{
public:

    Publisher(ros::NodeHandle &n,string name,int buff,const ClientIn& in,const ClientOut& out):
    mConnec(0),
    mIn(in),
    mOut(out)
    {
        init(n,name,buff);
    }
    Publisher(ros::NodeHandle &n,string name,int buff,const ClientIn& in):
    mConnec(0),
    mIn(in),
    mOut(nullptr)
    {
        init(n,name,buff);
    }
    Publisher(ros::NodeHandle &n,string name,int buff):
    mConnec(0),
    mIn(nullptr),
    mOut(nullptr)
    {
        init(n,name,buff);
    }
    ~Publisher()
    {

    }
    void subcriberConnect(const ros::SingleSubscriberPublisher& sub)
    {  
        int cnt=++mConnec;
        if(mIn && cnt>0){
            plt_assert(cnt>0);
            mIn(cnt);   
        }
    }
    void disconnectConnect(const ros::SingleSubscriberPublisher& sub)
    {
        if ("motion" == mName){
            return;
        }

        int cnt=--mConnec;
        if(mOut){
            plt_assert(cnt>=0);
            mOut(cnt);   
        }
    }
    void unSubConnect()
    {
        int cnt=--mConnec;
        if(mOut && cnt>=0){
            plt_assert(cnt>=0);
            mOut(cnt);   
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
         const ros::SubscriberStatusCallback conCB=bind(&Publisher::subcriberConnect,this,placeholders::_1);
         const ros::SubscriberStatusCallback disCB=bind(&Publisher::disconnectConnect,this,placeholders::_1);
         mPub=n.advertise<T>(name,buff,conCB,disCB);
     }
     string mName;
    ros::Publisher mPub;
    atomic<int> mConnec;
    ClientIn mIn;
    ClientOut mOut;
};
}
#endif