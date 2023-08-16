#ifndef __ROLLER_EYE_SINGLE_CLASS_H__
#define __ROLLER_EYE_SINGLE_CLASS_H__
#include<mutex>

using namespace std;
namespace roller_eye{
template<class T>
class SingleClass{
public:
    static T* getInstance()
    {
        lock_guard<mutex> lock(mMutex);
        if(mInst==nullptr){
            mInst=new T();
        }
        return mInst;
    }

protected:
    SingleClass()
    {

    }
    ~SingleClass()
    {

    }

    static T* mInst;
    static mutex mMutex;
};
template<class T>
T*  SingleClass<T>::mInst=nullptr;
template<class T>
mutex SingleClass<T>::mMutex;
}
#endif