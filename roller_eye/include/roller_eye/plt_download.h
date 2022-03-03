#ifndef __PLT_DOWNLOAD___H__
#define __PLT_DOWNLOAD___H__
#ifdef __cplusplus
extern "C"{
#endif

typedef void(*plt_send)(void* data,int size,int pos,void* args);
typedef void* PLTDLHandle;

struct plt_donwload
{
    const char* path;
    int blockSize;
    plt_send sender;
    void *args;
};

PLTDLHandle plt_donwload_create(struct plt_donwload* par);

int plt_donwload_seek(PLTDLHandle handle,int pos);

int plt_donwload_resume(PLTDLHandle handle);

int plt_donwload_pause(PLTDLHandle handle);
int plt_donwload_destroy(PLTDLHandle handle);

#ifdef __cplusplus
}
#endif
#endif