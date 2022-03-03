/*
 * Copyright (c) 2018, Fuzhou Rockchip Electronics Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _RKISP_CONTROL_LOOP_H_
#define _RKISP_CONTROL_LOOP_H_

#ifdef ANDROID_VERSION_ABOVE_8_X
#include <CameraMetadata.h>
using ::android::hardware::camera::common::V1_0::helper::CameraMetadata;
#else
struct camera_metadata;
typedef struct camera_metadata camera_metadata_t;
#endif
#ifdef __cplusplus
extern "C" {
#endif

#define RKISP_SENSOR_ATTACHED_FLASH_MAX_NUM 2
/*
 * This file is exposed by rkisp control loop(using rkisp CL instead below) library.
 * rkisp CL is the interfaces aggregation to control the ISP, sensor, lens,
 * flashlight ... settings as user required.
 * Example control flow:
 * 1. rkisp_cl_init
 *    This function is used to get the CL runtime context, and SHOULD be called
 *    before any other functions. The context is valid until rkisp_cl_deinit is
 *    called and returned.
 * 2. rkisp_cl_prepare
 *    After CL is initialized, some running parameters should be configured to
 *    CL before starting. These parameters are include various device paths.
 *    This method can only be called in CL stopped state.
 * 3. rkisp_cl_start
 *    When CL is prepared, then we can start the CL. After this methord is
 *    returned, the CL is running in its own thread. It gets statistics from
 *    driver, runs 3A algorimth to generate new ISP parameters ...
 * 4. rkisp_cl_set_frame_params
 *    This method can be called before step 3 which is used to set the
 *    initialized parameters, and should be called for every new frame request.
 * 5. cl_result_callback_ops
 *    It's the result callback method for the control loop to call into the hal.
 *    After step 4 and CL complete the calculation for 3a, CL will invoke the
 *    callback ops to give back the parameters corresponded to this frame.
 * 6. rkisp_cl_stop
 *    Call this to stop the CL. Then we can prepare CL again or deinit the CL.
 * 7. rkisp_cl_deinit
 *    Deinit the CL context returned by |rkisp_cl_init|. After this call is
 *    returnd, the |cl_ctx| is no longer valid.
 */

/* A struct used to set the preparing parameters for function
 * |rkisp_cl_prepare|
 */
struct rkisp_cl_prepare_params_s {
  // isp subdev node path
  const char* isp_sd_node_path;
  // isp params video node path
  const char* isp_vd_params_path;
  // isp statistics video node path
  const char* isp_vd_stats_path;
  // camera sensor subdev node path
  const char* sensor_sd_node_path;
  // lens subdev node path
  const char* lens_sd_node_path;
  // flashlight subdev node path
  const char* flashlight_sd_node_path[RKISP_SENSOR_ATTACHED_FLASH_MAX_NUM];
  // static metadata
  const camera_metadata_t *staticMeta;
  // TODO: sensor mode descriptor and others
};

/* A struct used to represent the new parameters set to CL
 * and the result metadata retunred from CL
 */
struct rkisp_cl_frame_metadata_s {
    //frame id
    int id;
    // TODO: use camera_metadata from Android directly ?
    const camera_metadata_t *metas;
};

/*
 * Callback methods for the control loop to call into the hal.
 * CL return back the effective settings and 3A status meta rusults
 * for specific frame to hal
 * Args:
 *    |ops|: the hook to get the implemention instance of the interface
 *    |result|: frame metadatas applied for quried frame and 3A results.
 *             |result| may be NOT valid after this method is returned,CL is
 *             responsible for the allocation and release of the result buffer
 *
 * Returns:
 *    -EINVAL: failed
 *    0      : success
 */
typedef struct cl_result_callback_ops {

    void (*metadata_result_callback)(const struct cl_result_callback_ops *ops,
                                     struct rkisp_cl_frame_metadata_s *result);
} cl_result_callback_ops_t;

/*
 * Get the control loop context
 * Args:
 *    |cl_ctx|: if initialization is successful, |*cl_ctx| will be filled by
 *              CL library.
 *    |tuning_file_path|: tuning file used by 3A algorithm library.
 * Returns:
 *    -EINVAL: failed
 *    0      : success
 */
int rkisp_cl_init(void** cl_ctx, const char* tuning_file_path,
                  const cl_result_callback_ops_t *callback_ops);

/*
 * Prepare the neccesary conditions before CL running
 * Args:
 *    |cl_ctx|: current CL context
 *    |prepare_params|: params to be configured to CL library. |prepare_params|
 *                      may be invalid after the methord is returned.
 * Returns:
 *    -EINVAL: failed
 *    0      : success
 */

int rkisp_cl_prepare(void* cl_ctx,
                     const struct rkisp_cl_prepare_params_s* prepare_params);

/*
 * Starting the CL. After this function is called, the control loop is running.
 * It means that the CL is repeating the follow procedules:
 * 1) get statistics from isp subdev
 * 2) configure the new settings from user
 * 3) run 3A algorithm and get the result
 * 4) set the new settings to ISP, sensor, lens, flashlight...
 * Args:
 *    |cl_ctx|: current CL context
 * Returns:
 *    -EINVAL: failed
 *    0      : success
 */
int rkisp_cl_start(void* cl_ctx);

/*
 * Set new frame settings for specific request.
 * Args:
 *    |cl_ctx|: current CL context
 *    |frame_params|: new frame settings to be applied for specific request.
 *                    if null, it means there is no new settings, and last
 *                    frame settings will be used for new frame. |frame_params|
 *                    may be NOT valid after this method is returned.
 * Returns:
 *    -EINVAL: failed
 *    0      : success
 */
int rkisp_cl_set_frame_params(const void* cl_ctx,
                              const struct rkisp_cl_frame_metadata_s* frame_params);

/*
 * Stop the current control loop.
 * Args:
 *    |cl_ctx|: current CL context
 * Returns:
 *    -EINVAL: failed
 *    0      : success
 */
int rkisp_cl_stop(void* cl_ctx);

/*
 * deinit the current control loop.
 * Args:
 *    |cl_ctx|: current CL context
 * Returns:
 */
void rkisp_cl_deinit(void* cl_ctx);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // _RKISP_CONTROL_LOOP_H_
