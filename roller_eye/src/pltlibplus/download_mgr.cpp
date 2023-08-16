#include"download_mgr.h"
#include"plog_plus.h"
#include"plterrno.h"
namespace roller_eye{
    const char* DL_TAG="dl_mgr";
    DownloadSession::DownloadSession(int8_t src,uint32_t seq,uint32_t session,int blockSize,string& path,plt_send sender):
    mSrc(src),mSeq(seq),mSession(session),mSender(sender)
    {
        setEOF(false);
        resetTimeout();
        plt_donwload par={path.c_str(),blockSize,DownloadSessionMgrSendData,this};
        mHandle=plt_donwload_create(&par);
    }
    DownloadSession::~DownloadSession()
    {
        thread th([this](){
            if(plt_donwload_destroy(mHandle)<0){
                PLOG_ERROR(DL_TAG,"destroy downlad fail\n");
            }
        });
        th.join();
    }
    DownloadSession::operator bool()
    {
        return mHandle!=NULL;
    }
    uint32_t DownloadSession::getSession()
    {
        return mSession;
    }
    uint32_t DownloadSession::getSeq()
    {
        return mSeq;
    }
    int DownloadSession::retry(int start)
    {
        return plt_donwload_seek(mHandle,start);
    }
    uint8_t DownloadSession::getSRC()
    {
        return mSrc;
    }

     void DownloadSession::send(void* data,int size,int pos,void* args)
     {
         (*mSender)(data,size,pos,args);
     }
     void DownloadSession::resetTimeout()
     {
         mTimeout=30;
     }
    int DownloadSession::updateTimeout()
    {
        //PLOG_INFO(DL_TAG,"update timeout=%d\n",(int)mTimeout);
        if(mHandle==NULL){
            return -1;
        }
        if(mEOF){
            return --mTimeout;
        }else{
            return mTimeout;
        }
    }
     void DownloadSession::setEOF(bool eof)
     {
         mEOF=eof;
         resetTimeout();
     }
    shared_ptr<DownloadSession> DownloadSessionMgr::findSession(int8_t src,uint32_t seq,int session)
    {
        lock_guard<mutex> lock(mMutex);
        for(auto& v:mSessions){
            if(v->getSRC()==src&&v->getSeq()==seq&&v->getSession()==(uint32_t)session){
                return v;
            }
        }  
        return nullptr; 
    }
    void DownloadSessionMgr::removeSession(shared_ptr<DownloadSession> session)
    {
        lock_guard<mutex> lock(mMutex);
        mSessions.remove(session);
        PLOG_INFO(DL_TAG,"remove,current session count = %d\n",(int)mSessions.size());
    }

     void DownloadSessionMgr::addSession(shared_ptr<DownloadSession> session)
     {
        lock_guard<mutex> lock(mMutex);
        mSessions.push_back(session);
        mCondi.notify_one();
        PLOG_INFO(DL_TAG,"add,current session count = %d\n",(int)mSessions.size());
     }
     void DownloadSessionMgr::sendData(void *data,int size,int pos,void* args)
     {
        auto s=static_cast<DownloadSession*>(args);
        auto ps=DownloadSessionMgr::getInstance()->findSession(s->getSRC(),s->getSeq(),s->getSession());
        if(ps){
            ps->setEOF(size<=0);
            ps->send(data,size,pos,args);
        }
     }
      void DownloadSessionMgr::update()
      {
            vector<shared_ptr<DownloadSession>> removeCache;
            while(true){
                removeCache.clear();
                this_thread::sleep_for(chrono::seconds(1));
                unique_lock<mutex> lock(mMutex);
                mCondi.wait(lock,[this]()->bool{
                    return !mSessions.empty();
                });
                for(auto& v:mSessions){
                    if(v->updateTimeout()<0){
                        PLOG_DEBUG(DL_TAG,"session[0x%08x:%d] expired\n",v->getSeq(),(int)v->getSRC());
                        removeCache.push_back(v);
                    }
                }
                for(auto& v:removeCache){
                    mSessions.remove(v);
                }
            }
      }
    DownloadSessionMgr::DownloadSessionMgr()
    {
        thread th([this](){
            update();
        });
        th.detach();
    }
    DownloadSessionMgr::~DownloadSessionMgr()
    {

    }
    void DownloadSessionMgrSendData(void *data,int size,int pos,void* args)
    {
        DownloadSessionMgr::getInstance()->sendData(data,size,pos,args);
    }
}