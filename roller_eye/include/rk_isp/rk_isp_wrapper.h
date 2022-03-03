#ifndef __RK_ISP_WRAPPER___
#define __RK_ISP_WRAPPER___

#ifdef __cplusplus
extern "C" {
#endif

void* rk_isp_create(const char* dev);
int rk_isp_start(void* ins);
int rk_isp_process(void* ins);
int rk_isp_stop(void* ins); 
void rk_isp_destory(void* ins);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif