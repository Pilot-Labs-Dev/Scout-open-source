#ifndef __ROLLER_EYE_DL_MGR__H__
#define __ROLLER_EYE_DL_MGR__H__

#include<string>
#include<thread>
#include<mutex>
#include<condition_variable>
#include<list>
#include<memory>
#include<atomic>
#include"plt_download.h"
#include"single_class.h"

using namespace std;
namespace roller_eye{
class DownloadSessionMgr;
class DownloadSession{
    friend class DownloadSessionMgr;
public:
    DownloadSession(int8_t src,uint32_t seq,uint32_t session,int blockSize,string& path,plt_send sender);
    ~DownloadSession();
    operator bool();
    uint32_t getSession();
    uint32_t getSeq();
    int retry(int start);
    uint8_t getSRC();
private:
    void send(void* data,int size,int pos,void* args);
    void resetTimeout();
    int updateTimeout();
    void setEOF(bool eof);
private:
    int8_t mSrc;
    uint32_t mSeq;
    uint32_t mSession;
    PLTDLHandle mHandle;
    plt_send mSender;
    atomic<int> mTimeout;
    atomic<bool> mEOF;
};

class DownloadSessionMgr:public SingleClass<DownloadSessionMgr>{
    friend class SingleClass<DownloadSessionMgr>;
public:
 
    shared_ptr<DownloadSession> findSession(int8_t src,uint32_t seq,int session);
    void removeSession(shared_ptr<DownloadSession> session);
    void addSession(shared_ptr<DownloadSession> session);
    void sendData(void *data,int size,int pos,void* args);
    void update();
private:
    DownloadSessionMgr();
    ~DownloadSessionMgr();

    mutex mMutex;
    condition_variable mCondi;
    list<shared_ptr<DownloadSession>> mSessions;
};
void DownloadSessionMgrSendData(void *data,int size,int pos,void* args);
}
#endif