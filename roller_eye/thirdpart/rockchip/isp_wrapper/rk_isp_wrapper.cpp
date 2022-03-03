/*
 * V4L2 video capture example
 * AUTHOT : Jacob Chen
 * DATA : 2018-02-25
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h> /* getopt_long() */
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <dlfcn.h>

#ifdef ANDROID
#include <drm.h>
#include <drm_mode.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#endif

#include <linux/videodev2.h>
#include <rkisp_control_loop.h>
#include <rkisp_dev_manager.h>
#include <interface/rkcamera_vendor_tags.h>
#include "mediactl.h"
#include"rk_isp_wrapper.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define FMT_NUM_PLANES 1

#define BUFFER_COUNT 4

#define RK_IQ_FILE  "/etc/cam_iq.xml"
#define LIBRKISP    "/usr/lib/librkisp.so"

enum io_method {
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
    IO_METHOD_DMABUF,
};

typedef int (*rkisp_init_func)(void** cl_ctx, const char* tuning_file_path,
                               const cl_result_callback_ops_t *callback_ops);
typedef int (*rkisp_prepare_func)(void* cl_ctx,
                     const struct rkisp_cl_prepare_params_s* prepare_params);
typedef int (*rkisp_start_func)(void* cl_ctx);
typedef int (*rkisp_stop_func)(void* cl_ctx);
typedef int (*rkisp_deinit_func)(void* cl_ctx);
typedef int (*rkisp_cl_set_frame_params_func)(const void* cl_ctx,
                     const struct rkisp_cl_frame_metadata_s* frame_params);

struct RKIspFunc {
    void* rkisp_handle;
    rkisp_init_func init_func;
    rkisp_prepare_func prepare_func;
    rkisp_start_func start_func;
    rkisp_stop_func stop_func;
    rkisp_deinit_func deinit_func;
    rkisp_cl_set_frame_params_func set_frame_params_func;
};

struct RKisp_media_ctl
{
    /* media controller */
    media_device *controller;
    media_entity *isp_subdev;
    media_entity *isp_params_dev;
    media_entity *isp_stats_dev;
    media_entity *sensor_subdev;
};

static const int silent=0;
#define DBG(...) do { if(!silent) printf(__VA_ARGS__); } while(0)
#define ERR(...) do { fprintf(stderr, __VA_ARGS__); } while (0)

struct control_params_3A
{
    /* used to receive current 3A settings and 3A states
     * place this at first place, so we cast ops back to base type
     */
    cl_result_callback_ops_t _result_cb_ops;
    /* used to set new setting to CL, used by _RKIspFunc.set_frame_params_func */
    rkisp_cl_frame_metadata_s _frame_metas;
    /* to manage the 3A settings, used by _frame_metas */
    CameraMetadata _settings_metadata;
    /* to manage the 3A result settings, used by metadata_result_callback */
    CameraMetadata _result_metadata;
    XCam::Mutex _meta_mutex;
};

typedef struct _RKISP_instance{
    void* rkISPengine;
    struct RKIspFunc rkISPfunc;
    struct control_params_3A *par3a;
    char dev[64];
}RKISP_instance;

static
void metadata_result_callback(const struct cl_result_callback_ops *ops,
                              struct rkisp_cl_frame_metadata_s *result)
{
    camera_metadata_entry entry;
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)ops;

    SmartLock lock(ctl_params->_meta_mutex);
    /* this will clone results to _result_metadata */
    ctl_params->_result_metadata = result->metas;
}

/*
 * construct the default camera settings, including the static
 * camera capabilities, infos.
 */
static void construct_default_metas(CameraMetadata* metas)
{
    int64_t exptime_range_ns[2] = {0,30*1000*1000};
    int32_t sensitivity_range[2] = {0,3200};
    uint8_t ae_mode = ANDROID_CONTROL_AE_MODE_ON;
    uint8_t control_mode = ANDROID_CONTROL_MODE_AUTO;
    uint8_t ae_lock = ANDROID_CONTROL_AE_LOCK_OFF;
    int64_t exptime_ns = 10*1000*1000;
    int32_t sensitivity = 1600;

    metas->update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE, exptime_range_ns, 2);
    metas->update(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, sensitivity_range, 2);
    metas->update(ANDROID_CONTROL_AE_MODE, &ae_mode, 1);
    metas->update(ANDROID_SENSOR_EXPOSURE_TIME, &exptime_ns, 1);
    metas->update(ANDROID_CONTROL_MODE, &control_mode, 1);
    metas->update(ANDROID_SENSOR_SENSITIVITY, &sensitivity, 1);
    metas->update(ANDROID_CONTROL_AE_LOCK, &ae_lock, 1);
}

static void init_3A_control_params(struct control_params_3A* &par3A)
{
    camera_metadata_t* meta;

    meta = allocate_camera_metadata(DEFAULT_ENTRY_CAP, DEFAULT_DATA_CAP);
    assert(meta);
    par3A = new control_params_3A();
    assert(par3A);
    par3A->_result_cb_ops.metadata_result_callback = metadata_result_callback;
    par3A->_settings_metadata = meta;
    construct_default_metas(&par3A->_settings_metadata);
    par3A->_frame_metas.id = 0;
    par3A->_frame_metas.metas =
        par3A->_settings_metadata.getAndLock();
    par3A->_settings_metadata.unlock(par3A->_frame_metas.metas);
}

static void deinit_3A_control_params(struct control_params_3A* &par3A)
{
    if (par3A )
        delete par3A ;
    par3A = NULL;
}

static int rkisp_get_meta_frame_id(void* &engine, int64_t& frame_id) {
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(RKCAMERA3_PRIVATEDATA_EFFECTIVE_DRIVER_FRAME_ID);
    if (!entry.count) {
        DBG("no RKCAMERA3_PRIVATEDATA_EFFECTIVE_DRIVER_FRAME_ID\n");
        return -1;
    }
    frame_id = entry.data.i64[0];
    DBG("meta frame id is %" PRId64 "\n", entry.data.i64[0]);

    return 0;
}

static int rkisp_get_meta_frame_sof_ts(void* &engine, int64_t& sof_ts) {
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(RKCAMERA3_PRIVATEDATA_FRAME_SOF_TIMESTAMP);
    if (!entry.count) {
        DBG("no RKCAMERA3_PRIVATEDATA_FRAME_SOF_TIMESTAMP\n");
        return -1;
    }
    sof_ts = entry.data.i64[0];
    DBG("meta frame timestamp is %" PRId64 "\n", entry.data.i64[0]);

    return 0;
}

// convenient interfaces of 3A, compatible with cifisp
static int rkisp_getAeTime(void* &engine, float &time)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_EXPOSURE_TIME);
    if (!entry.count)
        return -1;

    time = entry.data.i64[0] / (1000.0 * 1000.0 * 1000.0);
    DBG("expousre time is %f secs\n", time);

    return 0;
}

static int rkisp_getAeMaxExposureTime(void* &engine, float &time)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE);
    if (entry.count != 2)
        return -1;

    time = entry.data.i64[1] / (1000.0 * 1000.0 * 1000.0);
    DBG("expousre max time is %f secs\n", time);

    return 0;
}

static int rkisp_getAeGain(void* &engine, float &gain)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_SENSITIVITY);
    if (!entry.count)
        return -1;

    gain = (float)entry.data.i32[0] / 100;
    DBG("expousre gain is %f\n", gain);

    return 0;
}

static int rkisp_getAeMaxExposureGain(void* &engine, float &gain)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE);
    if (!entry.count)
        return -1;

    gain = entry.data.i32[1] / 100;
    DBG("expousre max gain is %f \n", gain);

    return 0;
}

static int rkisp_setAeMaxExposureTime(void* &engine, float time,struct RKIspFunc &_RKIspFunc,void* _rkisp_engine)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;

    int64_t exptime_range_ns[2] = {0,0};

    exptime_range_ns[1] = time * 1000 * 1000 * 1000;
    ctl_params->_settings_metadata.update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
                                          exptime_range_ns, 2);
    // should update new settings id
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);
    return 0;
}

static int rkisp_setAeMaxExposureGain(void* &engine, float gain,struct RKIspFunc &_RKIspFunc,void* _rkisp_engine)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;

    int32_t sensitivity_range[2] = {0,0};

    sensitivity_range[1] = gain * 100;
    ctl_params->_settings_metadata.update(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
                                          sensitivity_range, 2);
    // should update new settings id
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);
    return 0;
}

static int rkisp_setManualGainAndTime(void* &engine, float hal_gain, float hal_time,struct RKIspFunc &_RKIspFunc,void* _rkisp_engine)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;

    int64_t exptime_ns = hal_time * 1000 * 1000 * 1000;
    // set to manual mode
    uint8_t ae_mode = ANDROID_CONTROL_AE_MODE_OFF;
    // convert to ISO100 unit
    int32_t sensitivity = hal_gain * 100;

    ctl_params->_settings_metadata.update(ANDROID_SENSOR_SENSITIVITY, &sensitivity, 1);
    ctl_params->_settings_metadata.update(ANDROID_CONTROL_AE_MODE, &ae_mode, 1);
    ctl_params->_settings_metadata.update(ANDROID_SENSOR_EXPOSURE_TIME, &exptime_ns, 1);
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);

    return 0;
}

static int rkisp_setAeMode(void* &engine, HAL_AE_OPERATION_MODE mode,struct RKIspFunc &_RKIspFunc,void* _rkisp_engine)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    // set to manual mode
    uint8_t ae_mode = ANDROID_CONTROL_AE_MODE_OFF;

    if (mode == HAL_AE_OPERATION_MODE_AUTO) {
        ae_mode = ANDROID_CONTROL_AE_MODE_ON;
        ctl_params->_settings_metadata.update(ANDROID_CONTROL_AE_MODE, &ae_mode, 1);
    } else if (mode == HAL_AE_OPERATION_MODE_MANUAL) {
        camera_metadata_entry entry;
        SmartLock lock(ctl_params->_meta_mutex);

        entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_EXPOSURE_TIME);
        if (!entry.count)
            return -1;
        ctl_params->_settings_metadata.update(ANDROID_SENSOR_EXPOSURE_TIME, entry.data.i64, 1);

        entry = ctl_params->_result_metadata.find(ANDROID_SENSOR_SENSITIVITY);
        if (!entry.count)
            return -1;
        ctl_params->_settings_metadata.update(ANDROID_SENSOR_SENSITIVITY, entry.data.i32, 1);

        ctl_params->_settings_metadata.update(ANDROID_CONTROL_AE_MODE, &ae_mode, 1);
    } else {
        ERR("unsupported ae mode %d\n", mode);
        return -1;
    }
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);

    return 0;
}

enum HAL_AE_STATE {
    HAL_AE_STATE_UNSTABLE,
    HAL_AE_STATE_STABLE,
};

static int rkisp_getAeState(void* &engine, enum HAL_AE_STATE &ae_state)
{
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_CONTROL_AE_STATE);
    if (!entry.count)
        return -1;

    switch (entry.data.u8[0]) {
    case ANDROID_CONTROL_AE_STATE_SEARCHING :
        ae_state = HAL_AE_STATE_UNSTABLE;
        break;
    default:
        ae_state = HAL_AE_STATE_STABLE;
        break;
    }

    return 0;
}

static int rkisp_set3ALocks(void* &engine, int locks,struct RKIspFunc &_RKIspFunc,void* _rkisp_engine)
{
    // TODO: only support ae lock now
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;

    uint8_t ae_lock = locks & HAL_3A_LOCKS_EXPOSURE ?
        ANDROID_CONTROL_AE_LOCK_ON : ANDROID_CONTROL_AE_LOCK_OFF;

    ctl_params->_settings_metadata.update(ANDROID_CONTROL_AE_LOCK, &ae_lock, 1);
    ctl_params->_frame_metas.id++;
    ctl_params->_frame_metas.metas =
        ctl_params->_settings_metadata.getAndLock();
    ctl_params->_settings_metadata.unlock(ctl_params->_frame_metas.metas);

    _RKIspFunc.set_frame_params_func(_rkisp_engine,
                                     &ctl_params->_frame_metas);
    return 0;
}

static int rkisp_get3ALocks(void* &engine, int& curLocks)
{
    // TODO: only support ae lock now
    struct control_params_3A* ctl_params =
        (struct control_params_3A*)engine;
    camera_metadata_entry entry;

    SmartLock lock(ctl_params->_meta_mutex);

    entry = ctl_params->_result_metadata.find(ANDROID_CONTROL_AE_LOCK);
    if (!entry.count)
        return -1;

    if (entry.data.u8[0] == ANDROID_CONTROL_AE_LOCK_ON)
        curLocks |= HAL_3A_LOCKS_EXPOSURE;

    return 0;
}

#define MAX_MEDIA_INDEX 8
static struct media_device* rkisp_get_media_dev_by_vnode(const char* vnode)
{
    char sys_path[64];
    struct media_device *device = NULL;
    uint32_t nents, j, i = 0;
    FILE *fp;

    while (i < MAX_MEDIA_INDEX) {
        snprintf (sys_path, 64, "/dev/media%d", i++);
        fp = fopen (sys_path, "r");
        if (!fp)
          continue;
        fclose (fp);

        device = media_device_new (sys_path);

        /* Enumerate entities, pads and links. */
        media_device_enumerate (device);

        nents = media_get_entities_count (device);
        for (j = 0; j < nents; ++j) {
          struct media_entity *entity = media_get_entity (device, j);
          const char *devname = media_entity_get_devname (entity);
          if (NULL != devname) {
            if (!strcmp (devname, vnode)) {
                  goto out;
            }
          }
        }
        media_device_unref (device);
    }

out:
    return device;
}
#define RK_ISP_SUBDEV   "rkisp1-isp-subdev"
#define RK_ISP_INPUT_PARAM  "rkisp1-input-params"
#define RK_ISP_STATICS     "rkisp1-statistics"    
#define ASSERT_ISP_INST(ins)\
    if(ins==NULL)\
    {\
        return -1;\
    }\
    RKISP_instance *isp_inst=(RKISP_instance *)ins
int rk_isp_start(void* ins)
{
    ASSERT_ISP_INST(ins);
    unsigned int i;
    struct RKisp_media_ctl rkisp;

    if (isp_inst->rkISPfunc.prepare_func != NULL) 
    {
        struct rkisp_cl_prepare_params_s params={0};
        int nents;

        if ((rkisp.controller = rkisp_get_media_dev_by_vnode (isp_inst->dev))==NULL)
        {
            DBG("Can't find controller, maybe use a wrong video-node or wrong permission to media node");
            return  -1;
        }
        rkisp.isp_subdev =media_get_entity_by_name (rkisp.controller, RK_ISP_SUBDEV,strlen(RK_ISP_SUBDEV));
        rkisp.isp_params_dev =media_get_entity_by_name (rkisp.controller, RK_ISP_INPUT_PARAM,strlen(RK_ISP_INPUT_PARAM));
        rkisp.isp_stats_dev =media_get_entity_by_name (rkisp.controller, RK_ISP_STATICS,strlen(RK_ISP_STATICS));
        
        nents = media_get_entities_count (rkisp.controller);
        rkisp.sensor_subdev = NULL;
        for (i = 0; i < nents; ++i) {
            struct media_entity *e;
            const struct media_entity_desc *ef;

            e = media_get_entity(rkisp.controller, i);
            ef = media_entity_get_info(e);
            if (ef->type == MEDIA_ENT_T_V4L2_SUBDEV_SENSOR) 
            {
                rkisp.sensor_subdev = e;
                break;
            }
        }
        assert(rkisp.sensor_subdev);

        params.isp_sd_node_path = media_entity_get_devname (rkisp.isp_subdev);
        params.isp_vd_params_path = media_entity_get_devname (rkisp.isp_params_dev);
        params.isp_vd_stats_path = media_entity_get_devname (rkisp.isp_stats_dev);
        params.sensor_sd_node_path = media_entity_get_devname (rkisp.sensor_subdev);
        params.staticMeta=isp_inst->par3a->_settings_metadata.getAndLock();

        DBG("entity path[%s,%s,%s,%s]\n",params.isp_sd_node_path,params.isp_vd_params_path,params.isp_vd_stats_path,params.sensor_sd_node_path);
        /*
        // isp subdev node path
        params.isp_sd_node_path="/dev/v4l-subdev0";
        // isp params video node path
        params.isp_vd_params_path="/dev/video3";
        // isp statistics video node path
        params.isp_vd_stats_path="/dev/video2";
        // camera sensor subdev node path
        params.sensor_sd_node_path="/dev/v4l-subdev2";
        */
        isp_inst->rkISPfunc.prepare_func(isp_inst->rkISPengine, &params);
        isp_inst->par3a->_settings_metadata.unlock(params.staticMeta);
        media_device_unref (rkisp.controller);
    }
    else
    {
        DBG("prepare_func is null");
        return -1;
    }
    
    

    // set initial user params
    if (isp_inst->rkISPfunc.set_frame_params_func != NULL) 
    {
        isp_inst->rkISPfunc.set_frame_params_func(isp_inst->rkISPengine,&isp_inst->par3a->_frame_metas);
    }
    else
    {
        DBG("set_frame_params_func is null");
        return -1;
    }
    

    if (isp_inst->rkISPfunc.start_func != NULL) 
    {
        isp_inst->rkISPfunc.start_func(isp_inst->rkISPengine);
    }
    else
    {
        DBG("start_func is null");
        return -1;
    }

    return rkisp_setAeMode((void*&)isp_inst->par3a, HAL_AE_OPERATION_MODE_AUTO,isp_inst->rkISPfunc,isp_inst->rkISPengine);
}

int rk_isp_stop(void* ins)
{
   ASSERT_ISP_INST(ins);
    if(isp_inst->rkISPfunc.stop_func!=NULL)
    {
        return isp_inst->rkISPfunc.stop_func(isp_inst->rkISPengine);
    }
    else
    {
        DBG("stop_func is null");
        return -1;
    }
}

static int init_rkisp(void* ins)
{
    ASSERT_ISP_INST(ins);
    isp_inst->rkISPfunc.rkisp_handle = dlopen(LIBRKISP, RTLD_NOW);
    if (isp_inst->rkISPfunc.rkisp_handle == NULL) 
    {
        DBG("open %s failed\n", LIBRKISP);
        return -1;
    } 
    else
    {
        DBG("open isp library %s ok\n", LIBRKISP);
        isp_inst->rkISPfunc.init_func=(rkisp_init_func)dlsym(isp_inst->rkISPfunc.rkisp_handle, "rkisp_cl_init");
        isp_inst->rkISPfunc.prepare_func=(rkisp_prepare_func)dlsym(isp_inst->rkISPfunc.rkisp_handle, "rkisp_cl_prepare");
        isp_inst->rkISPfunc.start_func=(rkisp_start_func)dlsym(isp_inst->rkISPfunc.rkisp_handle, "rkisp_cl_start");
        isp_inst->rkISPfunc.stop_func=(rkisp_stop_func)dlsym(isp_inst->rkISPfunc.rkisp_handle, "rkisp_cl_stop");
        isp_inst->rkISPfunc.deinit_func=(rkisp_deinit_func)dlsym(isp_inst->rkISPfunc.rkisp_handle, "rkisp_cl_deinit");
        isp_inst->rkISPfunc.set_frame_params_func=(rkisp_cl_set_frame_params_func)dlsym(isp_inst->rkISPfunc.rkisp_handle,"rkisp_cl_set_frame_params");

        init_3A_control_params(isp_inst->par3a);
        if (isp_inst->rkISPfunc.init_func != NULL) 
        {
            return isp_inst->rkISPfunc.init_func(&isp_inst->rkISPengine, RK_IQ_FILE,(const cl_result_callback_ops_t *)isp_inst->par3a);
        }
        else
        {
            DBG("init_func is null");
            return -1;
        }
    }
}

static int uninit_rkisp(void* ins)
{
     ASSERT_ISP_INST(ins);
    deinit_3A_control_params(isp_inst->par3a);
    
    if (isp_inst->rkISPfunc.deinit_func != NULL)
    {
        if(isp_inst->rkISPfunc.deinit_func(isp_inst->rkISPengine)<0)
        {
            DBG("deinit_func failed");
        }
    }
    else
    {
        DBG("deinit_func is null");
        return -1;
    }
    
    if(isp_inst->rkISPfunc.rkisp_handle!=NULL)
    {
        return dlclose(isp_inst->rkISPfunc.rkisp_handle);
    }
    else
    {
        DBG("rkisp_handle is null");
        return -1;
    }
}

void* rk_isp_create(const char* dev)
{
    RKISP_instance* ins=(RKISP_instance*)calloc(1,sizeof(RKISP_instance));
    if(ins!=NULL){
        strncpy(ins->dev,dev,sizeof(ins->dev)-1);
    }
    if(init_rkisp(ins)<0)
    {
        uninit_rkisp(ins);
        free(ins);
        ins=NULL;
    }
    return ins;
}

int rk_isp_process(void* ins)
{
    return 0;
}

void rk_isp_destory(void* ins)
{
    uninit_rkisp(ins);
    free(ins);
}
