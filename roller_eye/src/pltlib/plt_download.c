#include<pthread.h>
#include<stdio.h>
#include<string.h>
#include<sys/select.h>
#include<sys/time.h>
#include"plog.h"
#include"plterrno.h"
#include"plt_malloc.h"
#include"msgqueue.h"
#include"plt_download.h"


struct plt_download_s
{
    FILE *file;
    char *buff;
    int buffSize;
    QHandle queue;
    plt_send send;
    void* args;
    int pos;
    int status;
    pthread_t thread;
};

enum{
    MSG_DL_SET_START,
    MSG_DL_PAUSE,          
    MSG_DL_RESUME,
    MSG_DL_QUIT             
};
enum{
    dl_status_running,
    dl_status_stop,
    dl_status_pause,
    dl_status_error
};

#define DL_ASSERT_HANDLE(handle)\
    struct plt_download_s *dl=(struct plt_download_s *)handle;\
    if(dl==NULL){\
        pset_errno(PEPAR);\
        return -1;\
    }

static const char* DL_TAG="download";


static void msg_proc(struct pqueue_msg *msg )
{
    struct plt_download_s *dl=msg->parc;
    switch (msg->msg)
    {
    case MSG_DL_SET_START:
        if(msg->pars>=0){
            clearerr(dl->file);
            if(fseek(dl->file,msg->pars*dl->buffSize,SEEK_SET)<0){
                dl->status=dl_status_error;  
            }else{
                dl->pos=msg->pars;
            }
        }
        break;
    case MSG_DL_PAUSE:
        dl->status=dl_status_pause;
        break;
    case MSG_DL_RESUME:
        dl->status=dl_status_running;
        break;
    case MSG_DL_QUIT:
        dl->status=dl_status_stop;
        break;
    default:
        break;
    }
}
static void* dl_loop(void* args)
{
    int tmp,waiting=0;
    struct plt_download_s *dl=(struct plt_download_s *)args;

    while(1){
        if(waiting){
           if(pqueue_pop_timeout(dl->queue,waiting)<0){
               break;
           }
            waiting=0;
        }else{
            pqueue_pop(dl->queue);
        }

        if(dl->status==dl_status_error){
            (*dl->send)(NULL,-1,dl->pos,dl->args);
            break;
        }else if(dl->status== dl_status_pause){
            waiting=-1;
            continue;
        }else if(dl->status == dl_status_stop){
            break;
        }

        tmp=fread(dl->buff,1,dl->buffSize,dl->file);
        if(tmp==dl->buffSize|| (tmp>0&&feof(dl->file))){
            (*dl->send)(dl->buff,tmp,dl->pos++,dl->args);
        }else{
            if(feof(dl->file)){
                (*dl->send)(NULL,0,dl->pos,dl->args);
                waiting=-1;
            }else{
                (*dl->send)(NULL,-1,dl->pos,dl->args);
                    break;
            }
        }   
    }

    PLOG_DEBUG(DL_TAG,"thread end\n");
    return NULL;   
}
PLTDLHandle plt_donwload_create(struct plt_donwload* par)
{
    int code;
    if(par==NULL||par->blockSize<=0||par->blockSize>1024*1024||par->sender==NULL){
        pset_errno(PEPAR);
        return NULL;
    }
    struct plt_download_s *dl=plt_calloc(1,sizeof(*dl));
    if(dl==NULL){
        return NULL;
    }
    
    if((dl->file=fopen(par->path,"rb"))==NULL){
        pset_errno(PESYS);
        goto err_file;
    }
    if((dl->buff=(char*)plt_malloc(par->blockSize))==NULL){
        goto err_buff;
    }
    dl->buffSize=par->blockSize;
    if((dl->queue=pqueue_create(512,msg_proc))==NULL){
        goto err_queue;
    }
    dl->send=par->sender;
    dl->args=par->args;

     if((code=pthread_create(&dl->thread,NULL,dl_loop,dl))!=0){
        pset_pthreadno(code);
        goto err_thread;
    }
    return dl;

err_thread:
    pqueue_destroy(dl->queue);
err_queue:
    plt_free(dl->buff);
err_buff:
    fclose(dl->file);
err_file:
    plt_free(dl);
    return NULL;
}

int plt_donwload_seek(PLTDLHandle handle,int pos)
{
    DL_ASSERT_HANDLE(handle);
    struct pqueue_msg msg;

    msg.msg=MSG_DL_SET_START;
    msg.pars=pos;
    msg.parc=dl;

    return pqueue_push(dl->queue,msg,1);
}
int plt_donwload_resume(PLTDLHandle handle)
{
    DL_ASSERT_HANDLE(handle);
    struct pqueue_msg msg;

    msg.msg=MSG_DL_RESUME;
    msg.parc=dl;

    return pqueue_push(dl->queue,msg,1);
}
int plt_donwload_pause(PLTDLHandle handle)
{
     DL_ASSERT_HANDLE(handle);
    struct pqueue_msg msg;

    msg.msg=MSG_DL_PAUSE;
    msg.parc=dl;

    return pqueue_push(dl->queue,msg,1);
}
int plt_donwload_destroy(PLTDLHandle handle)
{
    int ret;
    DL_ASSERT_HANDLE(handle);

    struct pqueue_msg msg;
    msg.msg=MSG_DL_QUIT;
    msg.parc=dl;
    if(pqueue_push(dl->queue,msg,1)<0){
        return -1;
    }

    if((ret=pthread_join(dl->thread,NULL))!=0){
        pset_pthreadno(ret);
        return -1;
    }
    pqueue_destroy(dl->queue);
    plt_free(dl->buff);
    fclose(dl->file);
    plt_free(dl);
    return 0;
}