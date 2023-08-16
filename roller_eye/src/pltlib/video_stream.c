#include<pthread.h>
#include<stdlib.h>
#include<string.h>
#include<fcntl.h>
#include<unistd.h>
#include<errno.h>
#include<assert.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include"plterrno.h"
#include"plog.h"
#include "video_stream.h"

#ifndef MEDIA_USE_USB_CAMERA
#define USE_ROCKCHIP_ISP_CAMERA  //for rk isp
#endif

#if !defined(APP_ARCH_X86)
#define USE_ROCKCHIP_CAMERA
#endif

#if !defined(APP_ARCH_X86)&&defined(USE_ROCKCHIP_ISP_CAMERA)
#define SUPPORT_ROCKCHIP_ISP
#include "rk_isp_wrapper.h"
#include "rkisp_control_loop.h"
#endif

#define LOGI(fmt,...)
#define LOGD(fmt,...)   PLOG_WARN("videostream",fmt,##__VA_ARGS__)
#define LOGE(fmt,...)   do{pset_errno(PESYS);PLOG_ERROR("videostream",fmt,##__VA_ARGS__);}while(0)
#define LOGE_RET(fmt,code)   do{pset_errno(PESYS);PLOG_ERROR("videostream",fmt);return (code);}while(0)

typedef struct _FrameBuffCtl {
    FrameBuff buff;
    int refCount;
    int reclaimFlag;
    pthread_mutex_t mutex;
    struct _FrameBuffCtl *next;
}FrameBuffCtl;

typedef struct{
    FrameBuffCtl **head;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int size;
    int cap;
    int readIdx;
    int writeIdx;
}FrameQueue;

typedef struct _Camera Camera;

typedef struct _VideoStream{
    FrameQueue queue;
    Camera *camera;
    struct _VideoStream *next;
}VideoStream;

typedef struct _Camera{
    pthread_mutex_t ctlMutex;
    pthread_mutex_t freeMutex;
    pthread_cond_t freeCond;
    CaputreParam param;
    VideoStream *head;
    int streamCount;
    int devFd;
    int refCount;
    int threadFlag;//the caputure thread has started or not
    int destroyFlag;
    int capType;
    int isMulitPlane;
    FrameBuff *buff;
    FrameBuffCtl *freeCtl;
    FrameBuffCtl *usedCtl;
#ifdef SUPPORT_ROCKCHIP_ISP
    void* rkisp;
#endif
}Camera;

static int pu32(uint32_t val, char *buf,int bufsize,int base){
    if(bufsize <=32)
    {
        return -1;
    }
    buf[32]=0;
    bufsize=31;
	for(; bufsize >=0 ; --bufsize, val /= base)
		buf[bufsize] = "0123456789abcdef"[val % base];
	return 0;
}

static int openVideoCapDevice(const char *dev,int *isMultiplane)
{
    int fd ,ret,retryTimes=0;
    char cap_str[36];

    while((fd = open(dev,O_RDWR))< 0)
    {
        if( errno == EBUSY && retryTimes++<30)
        {
            usleep(100000);//100ms
            LOGD("Retry Open,times=%d\n",retryTimes);
            continue;
        }
        LOGE_RET("OpenDev\n",-1);
    }

    struct v4l2_capability cap;

    if((ret= ioctl(fd,VIDIOC_QUERYCAP,&cap))<0)
    {
        LOGE_RET("GetCap\n",-1);
    }

    pu32(cap.capabilities,cap_str,sizeof(cap_str),2);
    LOGI("cap info: driver=%s,card=%s,bus=%s,version=%d,cap=0x%08x,cap_str=%s,\n",cap.driver,cap.card,cap.bus_info,cap.version,cap.capabilities,cap_str);

    if(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
    {
        *isMultiplane=1;
    }
    else if(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    {
        *isMultiplane=0;
    }
    else
    {
        LOGE("Not Capture Device");
        close(fd);
        fd=-1;
    }
    return fd;
}

static int enumVideoFmt(int fd,int type)
{
    struct v4l2_fmtdesc desc;

    memset(&desc,0,sizeof(desc));

    desc.index = 0;
    desc.type =type;

    LOGI("support formats ------\n");
    while(ioctl(fd,VIDIOC_ENUM_FMT,&desc)!=-1)
    {
        desc.index++;

        LOGI("desc:%s,fmt:[%c,%c,%c,%c]\n",desc.description,(desc.pixelformat)&0xff,(desc.pixelformat>>8)&0xff,(desc.pixelformat>>16)&0xff,(desc.pixelformat>>24)&0xff);

    }
     LOGI("------\n");

    if(errno!=EINVAL)
    {
        LOGE_RET("EnumFmt\n",-1);
    }
    return 0;
}

static int selectInput(int fd,int index)
{
	if (ioctl(fd, VIDIOC_S_INPUT, &index)<0)
	{
		LOGE_RET("SelectInput\n",-1);
	}

    return 0;
}

static int setVideoParam(int fd,int type,int width,int height,uint32_t pixFmt,int fpsNum,int fpsDen)
{
    struct v4l2_format fmt;

    struct v4l2_streamparm parm;
    memset(&parm,0,sizeof(parm));
	parm.type = type;
	parm.parm.capture.capturemode =0;
	parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parm.parm.capture.timeperframe.numerator = fpsNum;
	parm.parm.capture.timeperframe.denominator = fpsDen;

	if (ioctl(fd, VIDIOC_S_PARM, &parm) < 0)
	{
#ifndef USE_ROCKCHIP_CAMERA
		LOGE_RET("SetFrameRate\n",-1);
#else
        LOGD("set frame rate fail\n");
#endif
	}

    memset(&fmt,0,sizeof(fmt));
    fmt.type = type;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height =height;
    fmt.fmt.pix.pixelformat = pixFmt;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if(ioctl(fd,VIDIOC_S_FMT,&fmt)<0)
    {
        LOGE_RET("SetFmt\n",-1);
    }
    return 0;
}

static int requestBuff(int fd,int type,int count)
{
    struct v4l2_requestbuffers req;

    memset(&req,0,sizeof(req));
    req.type = type;
    req.count=count;
    req.memory=V4L2_MEMORY_MMAP;

    LOGD("Request buff count=%d\n",count);

    int ret = ioctl(fd,VIDIOC_REQBUFS,&req);
    if(ret < 0)
    {
        LOGE_RET("MallocBuff\n",-1);
    }
    LOGD("requestBuff ret=%d\n",ret);

    return 0;
}

static int mapBuff(int fd,int type,int index,FrameBuff *frameBuff)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];

    memset(frameBuff,0,sizeof(*frameBuff));
    memset(&buf,0,sizeof(buf));

    buf.index = index;
    buf.type = type;
    buf.memory=V4L2_MEMORY_MMAP;

    if (type==V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
    {
        memset(planes, 0, sizeof(planes));
        buf.m.planes = planes;
        buf.length = VIDEO_MAX_PLANES;
    }

    if(ioctl(fd,VIDIOC_QUERYBUF,&buf) <0)
    {
        LOGE_RET("QueryBuff\n",-1);
    }

    if(type==V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        frameBuff->size = buf.length;
        frameBuff->addr = mmap(NULL,buf.length,PROT_READ|PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    }
    else
    {
        assert(buf.length==1);
        frameBuff->size = planes[0].length;
        frameBuff->addr = mmap(NULL,planes[0].length,PROT_READ|PROT_WRITE, MAP_SHARED, fd,  planes[0].m.mem_offset);
    }

    if(frameBuff->addr==MAP_FAILED)
    {
        frameBuff->addr=NULL;
        LOGE_RET("MapBuff\n",-1);
    }

    return 0;
}

static int pushBuff(int fd,int type,int index)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];

    memset(&buf,0,sizeof(buf));

    buf.index = index;
    buf.type = type;
    buf.memory=V4L2_MEMORY_MMAP;

    if (type==V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
    {
        memset(planes, 0, sizeof(planes));
        buf.m.planes = planes;
		buf.length = VIDEO_MAX_PLANES;
    }

    if(ioctl(fd,VIDIOC_QBUF,&buf) <0)
    {
        LOGE_RET("AddBuff\n",-1);
    }
    return 0;
}

static int pullBuff(int fd,int type,struct timeval* stamp)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];

    memset(&buf,0,sizeof(buf));

    buf.type = type;
    buf.memory=V4L2_MEMORY_MMAP;
    if (type==V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
    {
        memset(planes, 0, sizeof(planes));
        buf.m.planes = planes;
		buf.length = VIDEO_MAX_PLANES;
    }

    if(ioctl(fd,VIDIOC_DQBUF,&buf) <0)
    {
        LOGE_RET("PullBuff\n",-1);
    }
    *stamp=buf.timestamp;
    return buf.index;
}

static int startStream(int fd,int type)
{
    if(ioctl(fd,VIDIOC_STREAMON,&type) < 0)
    {
        LOGE_RET("StartStream\n",-1);
    }

    return 0;
}
/*
static int stopStream(int fd,int type)
{
    if(ioctl(fd,VIDIOC_STREAMOFF,&type) < 0)
    {
        LOGE_RET("StartStream\n",-1);
    }

    return 0;
}
*/
static int mallocMapBuff(Camera *camera)
{
    int i;
    FrameBuff *buf;
    CaputreParam *param=&camera->param;

    if((camera->buff==NULL) && (camera->buff = (FrameBuff*)calloc(param->v4l2BuffCount,sizeof(FrameBuff)))==NULL)
    {
        LOGD("Malloc Map Buff Fail\n");
        return -1;
    }

    for(i=0;i<param->v4l2BuffCount;i++)
    {
        buf=&camera->buff[i];
        if(mapBuff(camera->devFd,camera->capType,i,buf) < 0)
        {
            return -1;
        }

        buf->fInfo.width= param->sInfo.fInfo.width;
        buf->fInfo.height = param->sInfo.fInfo.height;
        buf->fInfo.fmt = param->sInfo.fInfo.fmt;

        if(pushBuff(camera->devFd,camera->capType,i) < 0)
        {
            return -1;
        }
    }

    return 0;
}

static void freeMapBuff(Camera *camera)
{
    int i;
    CaputreParam *param=&camera->param;

    //unmap all buff
    if(camera->buff!=NULL)
    {
        for(i=0;i<param->v4l2BuffCount;i++)
        {
            if(camera->buff[i].addr!=NULL)
            {
                 if(munmap(camera->buff[i].addr,camera->buff[i].size)<0)
                {
                    LOGE("Unmap");
                }
                camera->buff[i].addr=NULL;
            }
        }
        free(camera->buff);
        camera->buff=NULL;
    }
}

static int mallocBuffCtl(Camera *camera,int frameSize)
{
    int i;
    FrameBuffCtl *ctl;
    CaputreParam *param=&camera->param;

    LOGI("Frame Size = %d\n",frameSize);

    for(i=0;i<param->buffCount;i++)
    {
        if((ctl=(FrameBuffCtl*)calloc(1,sizeof(*ctl)))==NULL)
        {
            LOGD("Malloc Buff Ctl\n");
            break;
        }
        ctl->buff.size=frameSize;
        if((ctl->buff.addr=malloc(frameSize))==NULL)
        {
            LOGD("Malloc Buff Ctl Frame Buffer\n");
            free(ctl);
            break;
        }
        if(pthread_mutex_init(&ctl->mutex,NULL)!=0)
        {
            LOGD("Init Buff CTL Mutex fail\n");
            free(ctl->buff.addr);
            free(ctl);
            break;
        }

        ctl->next=camera->freeCtl;
        camera->freeCtl=ctl;
    }

    if(i!=param->buffCount)
    {
        for(--i;i>=0;i--)
        {
            if(pthread_mutex_destroy(&camera->freeCtl->mutex)!=0)
            {
                LOGD("Destory Frame Ctl Fail\n");
            }
            ctl=camera->freeCtl;
            camera->freeCtl=camera->freeCtl->next;
            free(ctl->buff.addr);
            free(ctl);

        }
    }

    return i==param->buffCount?0:-1;
}
static int reclaimBuffCtl(FrameBuffCtl *ctl)
{
    int ret;
    if((ret=pthread_mutex_destroy(&ctl->mutex))!=0)
    {
        LOGD("Destory Frame Ctl Fail\n");
    }
    free(ctl->buff.addr);
    free(ctl);
    return ret;
}
static void freeBuffCtl(Camera *camera)
{
    FrameBuffCtl *ctl;

    if(pthread_mutex_lock(&camera->freeMutex)!=0)
    {
        LOGD("Lock camera free mutex\n");
    }
    //reclaim immediately
    while(camera->freeCtl!=NULL)
    {
        ctl=camera->freeCtl;
        camera->freeCtl=camera->freeCtl->next;
        reclaimBuffCtl(ctl);

    }
    //delay reclaim
    while(camera->usedCtl!=NULL)
    {
        camera->usedCtl->reclaimFlag=1;
        camera->usedCtl=camera->usedCtl->next;
    }
    pthread_mutex_unlock(&camera->freeMutex);
}

static int initCamera(Camera *camera)
{
    CaputreParam *param=&camera->param;

    if(camera->destroyFlag!=0)
    {
        LOGD("Camera Has Destoryed\n");
        return -1;
    }

    if((camera->devFd = openVideoCapDevice(param->videoDevPath,&camera->isMulitPlane))<0)
    {
        return -1;
    }
    camera->capType=(camera->isMulitPlane?V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:V4L2_BUF_TYPE_VIDEO_CAPTURE);
    LOGD("Open Camera Done,FD=%d,isMulitPlane=%d \n",camera->devFd,camera->isMulitPlane);

    enumVideoFmt(camera->devFd,camera->capType);


#ifndef USE_ROCKCHIP_CAMERA
    if(selectInput(camera->devFd,param->devIdx) <0 )
    {
        return -1;
    }
#endif

    if(setVideoParam(camera->devFd,camera->capType,param->sInfo.fInfo.width,param->sInfo.fInfo.height,param->sInfo.fInfo.fmt,param->sInfo.fpsNum,param->sInfo.fpsDen)<0)
    {
        return -1;
    }

    if(requestBuff(camera->devFd,camera->capType,param->v4l2BuffCount) < 0 )
    {
        return -1;
    }

    LOGD("requestBuff return \n");

    if(mallocMapBuff(camera)<0)
    {
        return -1;
    }

    if(mallocBuffCtl(camera,camera->buff[0].size)<0)
    {
        return -1;
    }

#ifdef SUPPORT_ROCKCHIP_ISP
    LOGD("rk_isp_start start\n");
    int ret = rk_isp_start(camera->rkisp);
    LOGD("rk_isp_start return %d\n", ret);
#endif

    if(startStream(camera->devFd,camera->capType) < 0)
    {
        return -1;
    }

    if(camera->param.hooks.ev!=NULL){
        (*camera->param.hooks.ev)(VIDEO_STREAM_EVENT_OPEN,camera->param.hooks.evPriv);
    }
    return 0;
}

static void stopCamera(Camera *camera)
{
#ifdef SUPPORT_ROCKCHIP_ISP
    rk_isp_stop(camera->rkisp);
#endif

    freeMapBuff(camera);

    LOGD("Stop Camera,FD=%d\n",camera->devFd);

    if(camera->devFd>=0)
    {
        close(camera->devFd);
        camera->devFd=-1;
        if(camera->param.hooks.ev!=NULL){
            (*camera->param.hooks.ev)(VIDEO_STREAM_EVENT_CLOSE,camera->param.hooks.evPriv);
        }
    }

    freeBuffCtl(camera);
}

static void video_stream_unref_camera(CMHandle handle)
{
    Camera *camera=(Camera*)handle;

    if(pthread_mutex_lock(&camera->ctlMutex)!=0)
    {
        LOGD("Lock Camera Fail\n");
        return;
    }
    if(camera->refCount==0)
    {
        LOGD("Can't reach here,bugs found\n");
    }

    if(--camera->refCount<=0)
    {
#ifdef SUPPORT_ROCKCHIP_ISP
        rk_isp_destory(camera->rkisp);
#endif
        LOGI("camera ref zero,destroy camera\n");

        pthread_mutex_unlock(&camera->ctlMutex);

        if(pthread_cond_destroy(&camera->freeCond)!=0)
        {
            LOGD("Destory Free Ctl Cond  Fail\n");
        }

        if(pthread_mutex_destroy(&camera->freeMutex)!=0)
        {
            LOGD("Destory Free Ctl Lock Fail\n");
        }

        if(pthread_mutex_destroy(&camera->ctlMutex)!=0)
        {
            LOGD("Destory Camera Lock Fail\n");
        }

        free(camera);
    }
    else
    {
        pthread_mutex_unlock(&camera->ctlMutex);
    }

}

static int video_stream_ref_camera(Camera *camera)
{
    if(pthread_mutex_lock(&camera->ctlMutex)!=0)
    {
        LOGD("Lock Camera Fail\n");
        return -1;
    }
    camera->refCount++;
    pthread_mutex_unlock(&camera->ctlMutex);
    return 0;
}

static void wakeupAllVideoStream(Camera *camera)
{
    VideoStream *stream;
    for(stream=camera->head;stream!=NULL;stream=stream->next)
    {
        if(pthread_mutex_lock(&stream->queue.mutex)!=0)
        {
            LOGD("Lock Video Stream Fail\n");
        }
        if(pthread_cond_signal(&stream->queue.cond)!=0)
        {
            LOGD("Signal Video Stream Fail\n");
        }
        pthread_mutex_unlock(&stream->queue.mutex);
    }
}
static int unrefBuffCtl(Camera *camera,FrameBuffCtl *ctl)
{
    int ret = 0;
    FrameBuffCtl *pre,*cur;

    if(ctl->refCount == 0)
    {
        LOGD("Can't reach here,bugs found\n");
        return -1;
    }
    if(--ctl->refCount == 0)
    {
        LOGI("return frame %p\n",ctl);
        if(pthread_mutex_lock(&camera->freeMutex)!=0)
        {
            LOGD("Lock Camera fail\n");
        }

        if(ctl->reclaimFlag==0)
        {
            for(cur=camera->usedCtl,pre=NULL;cur!=NULL;pre=cur,cur=cur->next)
            {
                if(cur==ctl)
                {
                    if(pre==NULL)
                    {
                        camera->usedCtl=camera->usedCtl->next;
                    }
                    else
                    {
                        pre->next=cur->next;
                    }
                    break;
                }
            }

            ctl->next=camera->freeCtl;
            camera->freeCtl=ctl;

            if(pthread_cond_signal(&camera->freeCond)!=0)
            {
                LOGD("Signal Free Ctl Cond Fail\n");
            }
        }
        else
        {
            ret=1;//need to be reclaimed;
        }

        pthread_mutex_unlock(&camera->freeMutex);
    }

    return ret;
}
static int returnBuffCtl(VideoStream *stream,FrameBuffCtl *ctl)
{
    int ret;

    Camera *camera=stream->camera;

    if(ctl==NULL)
    {
        return 0;
    }

    LOGI("unref buff %p from %p\n",ctl,stream);

    if(pthread_mutex_lock(&ctl->mutex)!=0)
    {
        LOGD("Lock Frame Buff Fail\n");
    }

    ret = unrefBuffCtl(camera,ctl);

    pthread_mutex_unlock(&ctl->mutex);

    if(ret==1)//reclaim
    {
        ret=reclaimBuffCtl(ctl);
    }

    return ret;
}
static void postAllVideoStreamFrame(Camera *camera,FrameBuffCtl *ctl)
{
    int ret;
    VideoStream *stream;
    FrameQueue *queue;

    if(pthread_mutex_lock(&ctl->mutex)!=0)
    {
        LOGD("Lock Frame Ctl Fail\n");
        return;
    }

    ctl->refCount=1;//avoid post fail

    if(pthread_mutex_lock(&camera->ctlMutex)==0)
    {
        for(stream=camera->head;stream!=NULL;stream=stream->next)
        {
            queue=&stream->queue;
            if(pthread_mutex_lock(&queue->mutex)!=0)
            {
                LOGD("Post Frame Fail to VideoStream  Fail\n");
                continue;
            }

            if(queue->head!=NULL)
            {
                if(queue->size==queue->cap)//buff full,overwrite first
                {
                    LOGI("stream video overwrite\n");
                    returnBuffCtl(stream,queue->head[queue->readIdx]);
                    queue->head[queue->readIdx]=NULL;
                    if(queue->readIdx==queue->cap-1)
                    {
                        queue->readIdx=0;
                    }
                    else
                    {
                        queue->readIdx++;
                    }
                    queue->size--;
                }

                {
                    queue->head[queue->writeIdx]=ctl;
                    if(queue->writeIdx==queue->cap-1)
                    {
                        queue->writeIdx=0;
                    }
                    else
                    {
                        queue->writeIdx++;
                    }
                    queue->size++;
                }
                ctl->refCount++;

                LOGI("Post Frame %p to %p\n",ctl,stream);

                if(pthread_cond_signal(&queue->cond)!=0)
                {
                    LOGD("Signal Queue Fail\n");
                }
            }
            else
            {
                LOGD("Post Frame %p to %p fail,Queue destroyed\n",ctl,stream);
            }

            pthread_mutex_unlock(&queue->mutex);
        }
        pthread_mutex_unlock(&camera->ctlMutex);
    }
    else
    {
        LOGD("Lock Camera Fail\n");
    }

    ret=unrefBuffCtl(camera,ctl);

    pthread_mutex_unlock(&ctl->mutex);

    if(ret==1)//never goes here
    {
        LOGD("Buff ctl free before caputure loop stopped,Bugs Found\n");
        reclaimBuffCtl(ctl);
    }
}
static FrameBuffCtl* getFreeFrame(Camera *camera)
{
    FrameBuffCtl *ctl;

    if(pthread_mutex_lock(&camera->freeMutex)!=0)
    {
        LOGD("Lock Camera Fail\n");
        return NULL;
    }

    if(camera->freeCtl==NULL)//not "while(camera->freeCtl==NULL)" if camera is destoryed,will wake up this
    {
        if(pthread_cond_wait(&camera->freeCond,&camera->freeMutex)!=0)
        {
            LOGD("Wait Free Ctl Signal Fail\n");
        }
    }

    if(camera->freeCtl!=NULL)
    {
        ctl=camera->freeCtl;
        camera->freeCtl=camera->freeCtl->next;
        ctl->refCount=0;
        ctl->next=camera->usedCtl;
        camera->usedCtl=ctl;
    }
    else
    {
        ctl=NULL;
    }

    pthread_mutex_unlock(&camera->freeMutex);

    return ctl;
}
static void* caputureCameraLoop(void* args)
{
    int idx;
    struct timeval stamp;
    FrameBuffCtl *ctl;
    Camera *camera=(Camera*)args;

    LOGD("Camera Caputure Thread Started...\n");

    pthread_detach(pthread_self());

    if(initCamera(camera)<0)
    {
        goto err;
    }

    while(1)
    {
        if(pthread_mutex_lock(&camera->ctlMutex)!=0)
        {
            LOGD("Lock Camera Fail\n");
        }

        LOGI("Capture One Frame ...\n");

        if(camera->head==NULL||camera->destroyFlag!=0)//all stream has been closed or called video_stream_destory_camera,stop caputre
        {
            goto stop;
        }
        else
        {
            pthread_mutex_unlock(&camera->ctlMutex);
            if((idx=pullBuff(camera->devFd,camera->capType,&stamp))<0)
            {
                goto err;
            }
            else
            {
                ctl=getFreeFrame(camera);
                if(ctl!=NULL)
                {
                    memcpy(&ctl->buff.fInfo,&camera->buff[idx].fInfo,sizeof(struct _FrameInfo));
                    memcpy(&ctl->buff.stamp,&stamp,sizeof(struct timeval));
                    if(camera->param.hooks.cp==NULL){
                        memcpy(ctl->buff.addr,camera->buff[idx].addr,ctl->buff.size);
                    }else{
                        (*camera->param.hooks.cp)(ctl->buff.addr,camera->buff[idx].addr,ctl->buff.size,&ctl->buff.fInfo,camera->param.hooks.priv);
                    }
                    postAllVideoStreamFrame(camera,ctl);
                }
                else
                {
                    LOGD("Get None Frame Ctl,Lost One Frame");
                }

                if(pushBuff(camera->devFd,camera->capType,idx)<0)
                {
                    goto err;
                }
            }
        }
    }

err:
    if(pthread_mutex_lock(&camera->ctlMutex)!=0)
    {
        LOGD("Lock Camera Fail\n");
    }
stop:
    LOGD("Camera Caputure Thread End ...\n");
    stopCamera(camera);
    camera->threadFlag=0;
    wakeupAllVideoStream(camera);
    pthread_mutex_unlock(&camera->ctlMutex);
    video_stream_unref_camera(camera);
    return NULL;
}

static int startCamera(Camera *camera)
{
    pthread_t thread;

    if(camera->threadFlag !=0)
    {
        return 0;
    }

    camera->refCount++;
    camera->threadFlag=1;
    if(pthread_create(&thread,NULL,caputureCameraLoop,camera)!=0)
    {
        camera->refCount--;
        camera->threadFlag=0;
        LOGD("Create Caputure Loop Fail\n");
        return -1;
    }

    return 0;
}

CMHandle video_stream_create_camera(CaputreParam *param)
{
    Camera *handle;

    if(param==NULL)
    {
        LOGD("Param is null\n");
        return NULL;
    }
    if((handle=(Camera*)malloc(sizeof(Camera)))==NULL)
    {
        LOGD("Malloc Memory Fail\n");
        return NULL;
    }

    memset(handle,0,sizeof(*handle));

    if(pthread_mutex_init(&handle->ctlMutex,NULL)!=0)
    {
        LOGD("Init Ctl Mutex Fail\n");
        goto freeHandle;
    }

    if(pthread_mutex_init(&handle->freeMutex,NULL)!=0)
    {
        LOGD("Init Free Mutex Fail\n");
        goto desMutex;
    }

    if(pthread_cond_init(&handle->freeCond,NULL)!=0)
    {
        LOGD("Init Free Mutex Fail\n");
        goto desFreeMute;
    }

    //to do : need to check params
    memcpy(&handle->param,param,sizeof(*param));

    handle->devFd=-1;
    handle->refCount=1;
#ifdef SUPPORT_ROCKCHIP_ISP
    handle->rkisp=rk_isp_create(param->videoDevPath);
#endif
    return (CMHandle)handle;

desFreeMute:
    if(pthread_mutex_destroy(&handle->freeMutex)!=0)
    {
        LOGD("Destory Free Mutex Fail\n");
    }
desMutex:
    if(pthread_mutex_destroy(&handle->ctlMutex)!=0)
    {
        LOGD("Destory Ctl Mutex Fail\n");
    }
freeHandle:
    free(handle);

    return NULL;
}

void video_stream_destory_camera(CMHandle handle)
{
    Camera *camera=(Camera*)handle;

    if(camera==NULL)
    {
        LOGD("NULL camera Handle\n");
        return;
    }

    camera->destroyFlag=1;

    if(pthread_cond_signal(&camera->freeCond)!=0)
    {
        LOGD("Singal Free Ctl Fail!\n");
    }

    while(camera->threadFlag!=0)//wait unitl device is closed
    {
        usleep(15000);//about 15ms
    }
    video_stream_unref_camera(handle);
}

int video_stream_get_camera_param(CMHandle handle,CaputreParam *param)
{
    Camera *camera=(Camera*)handle;

    if(camera==NULL)
    {
        LOGD("NULL camera Handle\n");
        return -1;
    }

    if(param==NULL)
    {
        return 0;
    }

    memcpy(param,&camera->param,sizeof(*param));

    return 0;
}
int video_stream_get_camera_fd(CMHandle handle)
{
    int fd;
    Camera *camera=(Camera*)handle;

    if(camera==NULL)
    {
        LOGD("NULL camera Handle\n");
        return -1;
    }

    if(pthread_mutex_lock(&camera->ctlMutex)!=0)
    {
        LOGD("Lock camera fail\n");
        return -1;
    }

    fd = camera->devFd;

    pthread_mutex_unlock(&camera->ctlMutex);

    return fd;
}
static int initFrameQueue(FrameQueue *queue,int size)
{
    LOGD("initFrameQueue\n");
    memset(queue,0,sizeof(*queue));

    if(pthread_mutex_init(&queue->mutex,NULL)!=0)
    {
        LOGD("Init Queue Mutex Fail\n");
        return -1;
    }

    if(pthread_cond_init(&queue->cond,NULL)!=0)
    {
        LOGD("Init Queue Cond Fail\n");
        goto desMutex;
    }

    queue->size=0;
    queue->cap=size;
    queue->head=(FrameBuffCtl**)calloc(size,sizeof(FrameBuffCtl*));
    if(queue->head==NULL)
    {
        LOGD("Malloc Queue Fail\n");
        goto desCond;
    }
    return 0;
desCond:
    pthread_cond_destroy(&queue->cond);
desMutex:
    pthread_mutex_destroy(&queue->mutex);
    return -1;
}

static void destroyFrameQueue(VideoStream *stream)
{

    LOGD("destroyFrameQueue enter 0x%x\n", stream);
    int i;
    FrameQueue *queue=&stream->queue;
    if(pthread_mutex_lock(&queue->mutex)!=0)
    {
        LOGD("Lock Queue Fail\n");
    }
    for(i=0;i<queue->cap;i++)
    {
        returnBuffCtl(stream,queue->head[i]);
    }
//     stream->camera->threadFlag = 0;
//    if(pthread_cond_signal(&queue->cond)!=0)
//    {
//         LOGD("pthread_cond_signal Fail\n");
//    }
    free(queue->head);
    queue->head=NULL;
    if(pthread_mutex_unlock(&queue->mutex)!=0)  {
        LOGD("Unlock Queue Mutex Fail\n");
    }

    if(pthread_mutex_destroy(&queue->mutex)!=0) {
        LOGD("Destroy Queue Mutex Fail\n");
    }
    LOGD("Destroy Queue Mutex succ\n");

    if(pthread_cond_destroy(&queue->cond)!=0) {
        LOGD("Destroy Queue Cond Fail\n");
    }
}

static int attachVideoStream(VideoStream *stream,Camera *camera)
{
    int ret=-1;
    if(pthread_mutex_lock(&camera->ctlMutex)!=0)
    {
        LOGD("Lock Camera Fail\n");
        return -1;
    }
    if(camera->streamCount < MAX_VIDEO_STREAMS)
    {
        if((ret=startCamera(camera))==0)
        {
            stream->next=camera->head;
            camera->head=stream;
            stream->camera=camera;
            camera->streamCount++;
        }
    }
    else
    {
        LOGD("Too Many Streams\n");
    }

    pthread_mutex_unlock(&camera->ctlMutex);
    return ret;
}
static void detachVideoStream(VideoStream *stream)
{
    VideoStream *item,*pre;
    Camera *camera=stream->camera;
    if(pthread_mutex_lock(&camera->ctlMutex)!=0)
    {
        LOGD("Lock Camera Fail\n");
    }

    for(item=camera->head,pre=NULL;item!=NULL;pre=item,item=item->next)
    {
        if(item==stream)
        {
            if(pre==NULL)
            {
                camera->head=stream->next;
            }
            else
            {
                pre->next=stream->next;
            }
            break;
        }
    }

    stream->next=NULL;
    camera->streamCount--;

    pthread_mutex_unlock(&camera->ctlMutex);
}
VSHandle video_stream_create(CMHandle handle,int buffCount)
{
    Camera *camera=(Camera*)handle;
    VideoStream *stream;

    if(camera==NULL)
    {
        LOGD("NULL camera Handle\n");
        return NULL;
    }

    if(camera->destroyFlag!=0)
    {
        LOGD("camera had been Destroyed\n");
        return NULL;
    }

    if(video_stream_ref_camera(camera)<0)
    {
        return NULL;
    }

    if((stream=(VideoStream*)malloc(sizeof(VideoStream)))==NULL)
    {
        LOGD("Malloc Video Stream Fail\n");
        goto unref;
    }

    memset(stream,0,sizeof(*stream));

    if(buffCount<=0 || buffCount >= camera->param.buffCount)
    {
        buffCount=camera->param.defaultCount;
    }
    LOGI("Video Stream Buff Count=%d\n",buffCount);

    LOGD("initFrameQueue 0x%x start\n",stream);
    if(initFrameQueue(&stream->queue,buffCount)<0)
    {
        goto freeHandle;
    }
    LOGD("initFrameQueue 0x%x\n",stream);
    if(attachVideoStream(stream,camera)<0)
    {
        goto desQueue;
    }

    return (VSHandle)stream;
desQueue:
    destroyFrameQueue(stream);
freeHandle:
    free(stream);
unref:
    video_stream_unref_camera(camera);
    return NULL;
}

void video_stream_destory(VSHandle handle)
{
    VideoStream *stream=(VideoStream*)handle;

    if(stream==NULL){
        LOGD("NULL Video Stream\n");
        return ;
    }
    detachVideoStream(stream);
    destroyFrameQueue(stream);
    video_stream_unref_camera(stream->camera);
    free(stream);
}
FrameBuff* video_stream_get_frame(VSHandle handle)
{
    FrameQueue *queue;
    VideoStream *stream=(VideoStream*)handle;
    FrameBuffCtl *ret=NULL;
    if(stream==NULL)
    {
        LOGD("NULL Video Stream\n");
        return NULL;
    }

    queue=&stream->queue;
    if(pthread_mutex_lock(&queue->mutex)!=0)
    {
        LOGD("Lock Queue Fail\n");
        return NULL;
    }

    while(queue->size==0&&stream->camera->threadFlag!=0)
    {
        if(pthread_cond_wait(&queue->cond,&queue->mutex)<0) {
            break;
        } else {
            continue;
        }
    }

    if(queue->size!=0 && stream->camera->threadFlag!=0)
    {
        ret=queue->head[queue->readIdx];
        queue->head[queue->readIdx]=NULL;
        if(queue->readIdx==queue->cap-1)
        {
            queue->readIdx=0;
        }
        else
        {
            queue->readIdx++;
        }
        queue->size--;

        LOGI("%p get frame %p\n",stream,ret);
    }
    pthread_mutex_unlock(&queue->mutex);
    return (FrameBuff *)ret;
}
int video_stream_return_frame(VSHandle handle,FrameBuff *frame)
{
    VideoStream *stream=(VideoStream*)handle;
    if(stream==NULL)
    {
        LOGD("NULL Video Stream\n");
        return -1;
    }
    if(frame==NULL)
    {
        LOGD("NULL Frame\n");
        return 0;
    }
    return returnBuffCtl(stream,(FrameBuffCtl*)frame);
}


#ifdef SUPPORT_ROCKCHIP_ISP
// struct control_params_3A
// {
//     /* used to receive current 3A settings and 3A states
//      * place this at first place, so we cast ops back to base type
//      */
//     cl_result_callback_ops_t _result_cb_ops;
//     /* used to set new setting to CL, used by _RKIspFunc.set_frame_params_func */
//     rkisp_cl_frame_metadata_s _frame_metas;
//     /* to manage the 3A settings, used by _frame_metas */
//     camera_metadata _settings_metadata;
//     /* to manage the 3A result settings, used by metadata_result_callback */
//     camera_metadata _result_metadata;
//     XCam::Mutex _meta_mutex;
// };

// typedef struct _RKISP_instance{
//     void* rkISPengine;
//     struct RKIspFunc rkISPfunc;
//     struct control_params_3A *par3a;
//     char dev[64];
// }RKISP_instance;

#endif

int setAeMaxExposureTime(CMHandle handle, float time)
{
// #ifdef SUPPORT_ROCKCHIP_ISP
//     Camera *camera = (Camera*)handle;
//     RKISP_instance *instance = (RKISP_instance*)camera->rkisp;
//     struct control_params_3A* ctl_params = (struct control_params_3A*)instance->par3a;
//     int64_t exptime_range_ns[2] = {0,0};

//     exptime_range_ns[1] = time * 1000 * 1000 * 1000;
//     ctl_params->_settings_metadata.update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
//     exptime_range_ns, 2);
//     // should update new settings id
//     ctl_params->_frame_metas.id ;
//     ctl_params->_frame_metas.metas =
//     ctl_params->_settings_metadata.getAndLock();
//     ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

//     instance->rkISPfunc.set_frame_params_func(instance->rkISPengine,
//     &ctl_params->_frame_metas);
// #endif
    return 0;
}