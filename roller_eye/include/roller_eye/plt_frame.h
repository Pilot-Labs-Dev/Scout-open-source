#ifndef __PLT_FRAME___H__
#define __PLT_FRAME___H__
#include<stdint.h>
#ifdef __cplusplus
extern "C"{
#endif

#pragma pack( push,1)
struct PltFrameInfo{
    uint32_t mSeq;
    uint64_t mStamp;
    uint32_t mSession;
    int8_t mType;
    int8_t mFormat;
    int32_t mPar1;
    int32_t mPar2;
    int32_t mPar3;
    int32_t mPar4;
};
#pragma pack(pop)

#ifdef __cplusplus
}
#endif
#endif