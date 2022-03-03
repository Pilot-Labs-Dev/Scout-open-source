#ifndef __PLT_ERRNO__H___
#define __PLT_ERRNO__H___
#ifdef __cplusplus
extern "C"{
#endif
//#define LIB_UBUS
#define LIB_UV
enum{
    PESYS=1,
    PEMEM,
    PENULL,
    PEBUG,
    PEFULL,
    PEEMPTY,
    PEBUSY,
    PEINVALID,
    PEPAR,
    PEBLOB,
    PEEIXST,
    PUNKOWN,
    PEBAD,
    PETEST,
    PEUBUS_START=1000,
    PEUBUS_END=PEUBUS_START+1000,
    PETHD_START=PEUBUS_END+100,
    PETHD_END=PETHD_START+1000,
    PEUV_START=PETHD_END+100,
    PEUV_END=PEUV_START+1000
};
enum{
    PE0=-100,
    PE1,
    PE2,
    PE3,
    PE4,
    PE5,
    PE6,
    PE7,
    PE8,
    PE9,
};
void pset_errno(int no);
#ifdef LIB_UBUS
void pset_ubusno(int no);
#endif
#ifdef LIB_UV
void pset_uvno(int no);
#endif
void pset_pthreadno(int no);
int pget_errno(void);
const char* pstr_errno(int no);
const char* pstr_lasterr(void);

#define PERRMSG_TEST       "Test Error Msg"
#ifdef __cplusplus
}
#endif
#endif