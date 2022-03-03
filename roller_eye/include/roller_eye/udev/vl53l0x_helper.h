#ifndef __ROLLER_EYE_VL53L0X_HELPER__H__
#define __ROLLER_EYE_VL53L0X_HELPER__H__
#include"vl53l0x_api.h"
#ifdef __cplusplus
extern "C" {
#endif

#define VL53L0X_CAP_MIN             0.03
#define VL53L0X_CAP_MAX            2.0
#define VL53L0X_CAP_ARC             0.43611111//25 degree

typedef enum {
	LONG_RANGE 		= 0, /*!< Long range mode */
	HIGH_SPEED 		= 1, /*!< High speed mode */
	HIGH_ACCURACY	= 2, /*!< High accuracy mode */
    NORMAL=3
} RangingConfig_e;

int tof_vl53l0x_init(VL53L0X_DEV tof);
int tof_vl53l0x_setup_single(VL53L0X_DEV tof,RangingConfig_e rangingConfig);
int tof_single_get_distance(VL53L0X_DEV tof);

#ifdef __cplusplus
}
#endif

#endif