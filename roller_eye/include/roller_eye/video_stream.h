#ifndef __VIDEO_STREAM_H__
#define __VIDEO_STREAM_H__

#ifdef __cplusplus
extern "C"{
#endif

#include<stdint.h>
#include<linux/videodev2.h>

#define MAX_VIDEO_STREAMS   10

struct _FrameInfo{
    int width;
    int height;
    uint32_t fmt;
};

typedef struct _FrameBuff {
    void *addr;
    int size;
    struct _FrameInfo fInfo;
    struct timeval stamp;
}FrameBuff;

typedef struct _StreamInfo{
    struct _FrameInfo fInfo;
    int fpsNum;
    int fpsDen;
    int h264Quality;
    int cameraLight;
}StreamInfo;

typedef void(*stream_copy)(void* dst,void* src,int len,struct _FrameInfo* info,void *priv);
enum{
    VIDEO_STREAM_EVENT_OPEN,
    VIDEO_STREAM_EVENT_CLOSE
};
typedef void(*camera_event)(int,void* priv);
typedef struct _CaptureHooks
{
    stream_copy cp;
    void *priv;
    camera_event ev;
    void* evPriv;
}CaptureHooks;

typedef struct _CaputreParam{
    StreamInfo sInfo;
    int defaultCount;
    int buffCount;
    int v4l2BuffCount;
    char videoDevPath[64];
    int devIdx;
    CaptureHooks hooks;
}CaputreParam;

typedef  void* CMHandle; //camera handle
typedef  void* VSHandle; //Video stream handle

CMHandle video_stream_create_camera(CaputreParam *param);
void video_stream_destory_camera(CMHandle handle);
int video_stream_get_camera_param(CMHandle handle,CaputreParam *param);
int video_stream_get_camera_fd(CMHandle handle);

VSHandle video_stream_create(CMHandle handle,int buffCount);
void video_stream_destory(VSHandle handle);
FrameBuff* video_stream_get_frame(VSHandle handle);
int video_stream_return_frame(VSHandle handle,FrameBuff *frame);

int setAeMaxExposureTime(CMHandle handle, float time);

#ifdef __cplusplus
}
#endif
#endif
