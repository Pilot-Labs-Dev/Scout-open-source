#pragma once
#include<memory>
#include<thread>
#include"plt_assert.h"
#include"roller_eye/block_queue.h"
#include"roller_eye/publisher2.h"
#include "unistd.h"
#include <sys/syscall.h>

using namespace std;

#define MIN_FREQ    (1e-3)
namespace roller_eye{
template <class T,class D>
class DataPublisher2{
public:
    enum{
        DATA_PUBLISHER_RUN,
        DATA_PUBLISHER_STOP,
        DATA_PUBLISHER_QUIT
    };
    DataPublisher2(shared_ptr<ros::NodeHandle> n,string name,int buff,shared_ptr<D> data,double freq):
    mHandle(n),
    mPub(*n,name,buff,bind(&DataPublisher2::clientIn,this)),
    mRate(freq>MIN_FREQ?freq:MIN_FREQ),
    mData(data),
    mCmd(100),
    mThread([this,freq](){
        loop(freq);        
    })
    {
    }
    ~DataPublisher2()
    {
        sendCmd(DATA_PUBLISHER_QUIT);
        mThread.join();
    }

     void disconnectConnect()
     {
         clientOut();
     }

private:
    void loop(double freq)
    {
        int cmd;
        bool quit=false,running=false; 
        string name = mPub.getName();
        while(!quit){
            if(running){
                if(mCmd.popNoblock(cmd)==0){
                    if(cmd==DATA_PUBLISHER_RUN){
                        continue;
                    }
                    mData->stop();
                    running=false;
                    if(cmd==DATA_PUBLISHER_QUIT){
                        quit=true;
                    }
                }else{
                    T data;            
                    if(mData->setData(data)==0){
                        mPub.publish(data);                    
                    }else{
                        if(freq<MIN_FREQ){
                            usleep(20000);                            
                        }
                    }
                    if(freq>MIN_FREQ){                    
                        mRate.sleep();                    
                    }
                }
            }else{
                mCmd.pop(cmd);
                if(cmd==DATA_PUBLISHER_QUIT){
                    quit=true;
                }else if(cmd==DATA_PUBLISHER_RUN){
                    mData->start();
                    running=true;
                    if(freq>MIN_FREQ){
                        mRate.reset();
                    }
                }
            }
        } 
    }
    void sendCmd(int cmd)
    {
        mCmd.push(cmd);
    }
    void clientIn()
    {
        sendCmd(DATA_PUBLISHER_RUN);
    }

    void clientOut()
    {
        sendCmd(DATA_PUBLISHER_STOP);
    }

    shared_ptr<ros::NodeHandle> mHandle;
    Publisher2<T> mPub;
    ros::Rate mRate;
    shared_ptr<D> mData;
    BlockQueue<int> mCmd;
    thread mThread;
};
}