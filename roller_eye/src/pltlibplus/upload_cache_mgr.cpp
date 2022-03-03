#include<fcntl.h>
#include<unistd.h>
#include<stdio.h>
#include<sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include<cstring>
#include <fstream>
#include <iostream>
#include <sys/inotify.h>
#include<sys/select.h>
#include"roller_eye/plt_assert.h"
#include"roller_eye/plog_plus.h"
#include"roller_eye/plterrno.h"
#include"roller_eye/upload_cache_mgr.h"

#define CACHE_FILE_READ     0
#define CACHE_FILE_WRITE    1
#define MAX_CACHE_BUFF_SIZE     (4*1024*1024)

namespace roller_eye{
    static const string CACHE_ROOT_PATH="/userdata/roller_eye/cache/";
    static const string CACHE_ROOT_PATH_TMP="/userdata/roller_eye/cache_tmp/";
    static const string READY_ROOT_PATH="/userdata/roller_eye/ready/";
    static const string THUMB_ROOT_PATH="/userdata/roller_eye/thumb/";
    static const string THUMB_CACHE_ROOT_PATH="/userdata/roller_eye/thumb_cache/";
    static const string THUMB_READY_ROOT_PATH="/userdata/roller_eye/thumb_ready/";
    static const char FRAME_SPLIT_TAG         =0xAA;

    CacheFile::CacheFile(const string& name, const string& baseName, int op):mFile(NULL),
       mBaseFileName(baseName), mVersion(0xff)
    {
        int flag=O_RDWR;
        char mode[4]="rb";

        if(op==CACHE_FILE_WRITE){
           flag=O_WRONLY|O_EXCL|O_CREAT;
           mode[0]='w';
        }
        
        if((mFd=open(name.c_str(),flag,0600))<0){
            return;
        }
    
        if(lockf(mFd,F_TLOCK,0)<0){
            ::close(mFd);
            return;
        }

        mFile=fdopen(mFd,mode);
    }
     CacheFile::~CacheFile()
     {
        close();
     }
    int CacheFile::close()
    {
        int ret=0;
        if(mFile!=NULL){
            ret=fclose(mFile);
            mFile=NULL;
         }
         return ret;
    }
    int  CacheFile::read(Cache& cache)
    {
        char tag;
        cache_size_t len;

        if(mFile==NULL){
            pset_errno(PENULL);
            return -1;
        }

        if(mVersion!=CACHE_PROTOCAL_VERSION){
            if(fread(&mVersion,1,1,mFile)!=1){
                return feof(mFile)?CACHE_EOF:PE0;
            }
            if(mVersion!=CACHE_PROTOCAL_VERSION){
                pset_errno(PEINVALID);
                return -1;
            }   
        }
        if(fread(&tag,1,1,mFile)!=1){
            return feof(mFile)?CACHE_EOF:PE1;
        }
        if(tag!=FRAME_SPLIT_TAG){
            pset_errno(PEBUG);
            return -1;
        }
        if(fread(&tag,1,1,mFile)!=1){
            return feof(mFile)?CACHE_EOF:PE3;
        }
        if(fread(&cache.stamp,sizeof(cache.stamp),1,mFile)!=1){
           return feof(mFile)?CACHE_EOF:PE4;
        }
        if(fread(&len,sizeof(len),1,mFile)!=1){
           return feof(mFile)?CACHE_EOF:PE5;
        }
        if(len>MAX_CACHE_BUFF_SIZE||len<0){
            pset_errno(PEBUG);
            return -1;
        }else{
            cache.buff.resize(len);
        }
        if(fread(&cache.buff[0],1,len,mFile)!=len){
            return feof(mFile)?CACHE_EOF:PE6;
        }
        cache.type=tag;
        return CACHE_OK;
    }
    int CacheFile::write(Cache& cache)
    {
        return write(&cache.buff[0],cache.buff.size(),cache.type,cache.stamp);
    }
    int  CacheFile::write(void* buff,cache_size_t size,int type,cache_stamp_t stamp)
    {
         if(mFile==NULL){
            pset_errno(PENULL);
            return -1;
        }

        if(mVersion!=CACHE_PROTOCAL_VERSION){
            mVersion=CACHE_PROTOCAL_VERSION;
            if(fwrite(&mVersion,1,1,mFile)!=1){
                mVersion=0xff;
                return PE9;
            }
        }

        if(size>MAX_CACHE_BUFF_SIZE||size<0){
            pset_errno(PEPAR);
            return -1;
        }

        if(fwrite(&FRAME_SPLIT_TAG,1,1,mFile)!=1){
            return PE0;
        }
        char tag=(char)type;
        if(fwrite(&tag,1,1,mFile)!=1){
            return PE1;
        }
        if(fwrite(&stamp,sizeof(stamp),1,mFile)!=1){
            return PE2;
        }
        if(fwrite(&size,sizeof(size),1,mFile)!=1){
            return PE3;
        }
        if(size==0){
            return CACHE_OK;
        }
        if(fwrite(buff,1,size,mFile)!=size){
            return PE4;
        }

        mDataSize += size;
        ofstream file(THUMB_ROOT_PATH+mBaseFileName+".size", ios::trunc);
        file << mDataSize;
        return CACHE_OK;
    }

    bool CacheFile::ok()
    {
        return mFile!=NULL;
    }
    CacheMgr::CacheMgr():mNofity(-1),mWatch(-1),mThumbReadyWatch(-1)
    {
        plt_assert(createDir(CACHE_ROOT_PATH.c_str())==0);
        plt_assert(createDir(READY_ROOT_PATH.c_str())==0);
        plt_assert(createDir(THUMB_READY_ROOT_PATH.c_str())==0);     
        plt_assert(createDir(THUMB_CACHE_ROOT_PATH.c_str())==0);                
        plt_assert(createDir(CACHE_ROOT_PATH_TMP.c_str())==0);      
        plt_assert(createDir(THUMB_ROOT_PATH.c_str())==0);      
    }
    int CacheMgr::createDir(const char* path)
    {
        if(access(path,F_OK)!=0)
        {
            if(mkdir(path,0755)<0){
                return (errno==EEXIST?0:-1);
            }
        }
        return 0;
    }
    shared_ptr<CacheFile> CacheMgr::newCache(const string& name,int type)
    {
        string path=CACHE_ROOT_PATH+name;
        auto cache=make_shared<CacheFile>(path, name, type);
        return !cache->ok()?nullptr:cache;
    }
    shared_ptr<CacheFile> CacheMgr::newReadCache(const string& name)
    {
        return newCache(name,CACHE_FILE_READ);
    }
    shared_ptr<CacheFile> CacheMgr::newWriteCache(const string& name)
    {
        return newCache(name,CACHE_FILE_WRITE);
    }
    int CacheMgr::testLock(const char* name)
    {
        int ret;
        int fd=open(name,O_RDWR);
        if(fd<0){
            return -1;
        }
        ret=lockf(fd,F_TLOCK,0);
        close(fd);
        return ret;
    } 
    int CacheMgr::removeThumb(const string&name)
    {
        string path=THUMB_ROOT_PATH+name;
        if(testLock(path.c_str())<0){
            return -1;
        }
        return unlink(path.c_str());
    }
    int CacheMgr::removePending(const string&name)
    {
        string path=CACHE_ROOT_PATH+name;
        if(testLock(path.c_str())<0){
            return -1;
        }
        return unlink(path.c_str());
    }
    int CacheMgr::removeReady(const string&name)
    {
        string path=READY_ROOT_PATH+name;
        return unlink(path.c_str());
    }
    int CacheMgr::setReady(const string& name)
    {
        string opath=CACHE_ROOT_PATH+name;
        string npath=READY_ROOT_PATH+name;
        if(testLock(opath.c_str())<0){
            return -1;
        }
        return rename(opath.c_str(),npath.c_str());
    }    
    int CacheMgr::removeThumbCache(const string&name)
    {
        string path=THUMB_CACHE_ROOT_PATH+name;
        return unlink(path.c_str());
    }
    int CacheMgr::removeThumbReady(const string&name)
    {
        string path=THUMB_READY_ROOT_PATH+name;
        return unlink(path.c_str());
    }
    int CacheMgr::setThumbReady(const string& name)
    {
        string opath=THUMB_CACHE_ROOT_PATH+name;
        string npath=THUMB_READY_ROOT_PATH+name;
        if(testLock(opath.c_str())<0){
            return -1;
        }
        return rename(opath.c_str(),npath.c_str());
    }
    int CacheMgr::getSizeFromThumbReady(const string& name)
    {
        std::ifstream in(THUMB_READY_ROOT_PATH+name.c_str());
        in.seekg(0, std::ios::end);
        size_t size = in.tellg();
        in.close();  
        return  size;
    }
    int CacheMgr::getSizeFromReady(const string& name)
    {
        std::ifstream in(READY_ROOT_PATH+name.c_str());
        in.seekg(0, std::ios::end);
        size_t size = in.tellg();
        in.close();  
        return  size;
    }

   vector<string> CacheMgr::scanThumbList()
    {
        return scanList(THUMB_ROOT_PATH,true);
    }
    vector<string> CacheMgr::scanThumbCacheList()
    {
        return scanList(THUMB_CACHE_ROOT_PATH,true);
    }
    vector<string> CacheMgr::scanThumbReadyList()
    {
        return scanList(THUMB_READY_ROOT_PATH,true);
    }
    vector<string> CacheMgr::scanPendingList()
    {
        return scanList(CACHE_ROOT_PATH,true);
    }
    vector<string> CacheMgr::scanReadyList()
    {
        return scanList(READY_ROOT_PATH,false);
    }
    vector<string> CacheMgr::scanList(const string& path,bool checkLock)
    {
        vector<string> names;
        DIR* dir=opendir(path.c_str());
        if(dir==NULL){
            return names;
        }
        struct dirent *entry;
        while((entry=readdir(dir))!=NULL){
            if(strcmp(entry->d_name,".")==0||strcmp(entry->d_name,"..")==0){
                continue;
            }
            if(checkLock){
                string p=path+entry->d_name;
                if(testLock(p.c_str())<0){
                    continue;
                }
            }
            names.push_back(entry->d_name);
        }
        closedir(dir);
        return names;
    }
    int CacheMgr::watch_start()
    {
        if(mNofity>=0){
            return 0;
        }
        if((mNofity=inotify_init1(O_NONBLOCK))<0){
            return -1;
        }
        if((mThumbReadyNofity=inotify_init1(O_NONBLOCK))<0){
            return -1;
        }
        
        //watch IN_CLOSE_WRITE  event is not the best method ,becuase CacheMgr::testLock will trigger this event
        //are there any better event to watch cache file is completed?
        if((mWatch=inotify_add_watch(mNofity,CACHE_ROOT_PATH.c_str(),IN_CLOSE_WRITE))<0){
            close(mNofity);
            mNofity=-1;
            return -1;
        }
        if((mThumbReadyWatch=inotify_add_watch(mThumbReadyNofity,THUMB_READY_ROOT_PATH.c_str(),IN_CLOSE_WRITE))<0){
            inotify_rm_watch(mNofity,mWatch);
            mWatch = -1;
            close(mNofity);
            mNofity=-1;
            return -1;
        }
        
        return 0;
    }
    int CacheMgr::watch_wait(bool& run)
    {
        int ret;
        char buf[256];
        struct timeval val;

        if(mNofity<0 || mThumbReadyNofity<0){
            return -1;
        }

        fd_set rd;
    
        while(run){
            val.tv_sec=5;val.tv_usec=0;
            FD_ZERO(&rd);
            FD_SET(mNofity,&rd);
            FD_SET(mThumbReadyNofity,&rd);
            int maxfd = mNofity>mThumbReadyNofity ? mNofity:mThumbReadyNofity;
            if((ret=select(maxfd+1,&rd,NULL,NULL,&val))<0){
                return -1;
            }
            if(ret==0){
                continue;
            }
              if(FD_ISSET(mNofity,&rd)){
                while((ret=read(mNofity,&buf,sizeof(buf)))>0);
              }
            if(FD_ISSET(mThumbReadyNofity,&rd)){
                while((ret=read(mThumbReadyNofity,&buf,sizeof(buf)))>0);
            }
            break;
        }
        return 0;
    }
    int CacheMgr::watch_end()
    {
        int ret=0;
        if(mNofity<0){
            return -1;
        }
        if(inotify_rm_watch(mNofity,mWatch)<0){
            ret=-1;
        }
        if(close(mNofity)<0){
            ret=-1;
        }
        mWatch=-1;
         if(inotify_rm_watch(mThumbReadyNofity,mThumbReadyWatch)<0){
            ret=-1;
        }
        mThumbReadyWatch = -1;
        if(close(mThumbReadyNofity)<0){
            ret=-1;
        }
        
        mNofity=-1;
        mThumbReadyNofity=-1;
        return ret;
    }
}