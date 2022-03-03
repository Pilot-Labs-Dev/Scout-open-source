#ifndef __ROLLER_EYE_UPLOAD_CACHE_MGR_H__
#define __ROLLER_EYE_UPLOAD_CACHE_MGR_H__
#include<string>
#include<vector>
#include<memory>
#include"roller_eye/single_class.h"
#include"roller_eye/plt_config.h"
#include<fstream>
#include<iostream>
#include<sstream>

#define CACHE_PROTOCAL_VERSION  0x01 

#define CACHE_EOF                       0
#define CACHE_OK                         1

enum{
CACHE_AUDIO_BUFF='A',
CACHE_VIDEO_BUFF='V',     
CACHE_VIDEO_I_BUFF='I',     
CACHE_VIDEO_P_BUFF='P'     
};

typedef uint32_t cache_stamp_t;
typedef uint32_t cache_size_t;

using namespace std;
namespace roller_eye{
    struct Cache{
        vector<uint8_t> buff;
        cache_stamp_t stamp;
        int type;  //'A','V','I','P'
    };
    class CacheFile{
    public:
        CacheFile(const string& name, const string& baseName, int op);
        ~CacheFile();
 
        int read(Cache& cache);

        int write(Cache& cache);
        int write(void* buff,cache_size_t size,int type,cache_stamp_t stamp);

        bool ok();
        int close();

    private:
        int mFd;
        FILE *mFile;
        string mBaseFileName;
        int mDataSize=0;
        unsigned char mVersion;
    };

    class CacheMgr:public SingleClass<CacheMgr>{
        friend class SingleClass<CacheMgr>;
    public:

        shared_ptr<CacheFile> newReadCache(const string& name);

        shared_ptr<CacheFile> newWriteCache(const string& name);

        int removeThumb(const string&name);


        int removePending(const string&name);
 
        int removeReady(const string&name);

        int setReady(const string& name);    

        int removeThumbCache(const string&name);
        int removeThumbReady(const string&name);
        int setThumbReady(const string& name);    

        vector<string> scanThumbList();
        vector<string> scanThumbCacheList();
        vector<string> scanThumbReadyList();
  
        int getSizeFromReady(const string& name);
        int getSizeFromThumbReady(const string& name);        
 
        vector<string> scanPendingList();

        vector<string> scanReadyList();

        int watch_start();
 
        int watch_wait(bool& run);

        int watch_end();
    private:
        CacheMgr();
        shared_ptr<CacheFile> newCache(const string& name,int type);
        vector<string> scanList(const string& path,bool checkLock);
        int testLock(const char* name);
        int createDir(const char* path);
        int mNofity;
        int mWatch;
        int mThumbReadyNofity;
        int mThumbReadyWatch;
    };
}
#endif