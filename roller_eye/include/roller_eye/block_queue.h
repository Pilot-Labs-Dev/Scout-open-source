#ifndef __ROLLER_EYE_BLOCK_QUEUE_H__
#define __ROLLER_EYE_BLOCK_QUEUE_H__
#include<mutex>
#include<queue>
#include <condition_variable>
#include"plterrno.h"

using namespace std;
namespace roller_eye{
 template<class T>
class BlockQueue{
public:
    BlockQueue(int max):mMax(max)
    {

    }
    ~BlockQueue()
    {

    }

    int pushNoblock(T& t)
    {
        unique_lock<mutex> lock(mMutex);
        if(mMax>0&&(int)mQueue.size()>=mMax){
            pset_errno(PEFULL);
            return -1;
        }
        mQueue.push(t);
        if(mQueue.size()==1){
            mCondR.notify_all();
        }
        return 0;
    }

     void push(T& t)
    {
        unique_lock<mutex> lock(mMutex);
        while(mMax>0&&(int)mQueue.size()>=mMax){
            mCondW.wait(lock);
        }
        mQueue.push(t);
        if(mQueue.size()==1){
            mCondR.notify_all();
        }
    }

    void pop(T& t)
    {
        unique_lock<mutex> lock(mMutex);
        while(mQueue.size()==0)
        {
            mCondR.wait(lock);
        }

        t = mQueue.front();
        mQueue.pop();

        if(mMax>0 && (int)mQueue.size()==mMax-1)
        {
            mCondW.notify_all();
        }
    }

    int popNoblock(T& t)
    {
        unique_lock<mutex> lock(mMutex);
        if(mQueue.size()==0)
        {
            pset_errno(PEEMPTY);
            return -1;
        }

        t = mQueue.front();
        mQueue.pop();

        if(mMax>0 && (int)mQueue.size()==mMax-1)
        {
            mCondW.notify_all();
        }
        return 0;
    }

    void clear(){
        unique_lock<mutex> lock(mMutex);
        while(!mQueue.empty()){
            mQueue.pop();
        }
        mCondW.notify_all();
    }

    bool empty(){
        unique_lock<mutex> lock(mMutex);
        return mQueue.empty();
    }
private:
    int mMax;
    mutex mMutex;
    condition_variable mCondW;
    condition_variable mCondR;
    queue<T> mQueue;
};
}
#endif