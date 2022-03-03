#include <sys/types.h>
#include <sys/stat.h>
#include<unistd.h>
#include<fcntl.h>
#include"vl53l0x_helper.h"
#include"plt_tools.h"

#define TOF_I2C_PATH        "/dev/i2c-1"

static const int DEFAULT_ADDR=0x52;

int tof_vl53l0x_init(VL53L0X_DEV tof)
{
    uint16_t id;
    VL53L0X_Error err;
    if(change_file_mode(TOF_I2C_PATH,"666")!=0){
        return -1;
    }
	if((tof->I2cHandle=open(TOF_I2C_PATH,O_RDWR))<0){
        return -1;
    }
	
    tof->I2cDevAddr=DEFAULT_ADDR;

    if((err=VL53L0X_RdWord(tof, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id))!=VL53L0X_ERROR_NONE){
        return -err;
    }
    
    if(id!=0xEEAA){
        return -2;
    }

    if((err=VL53L0X_DataInit(tof))!=VL53L0X_ERROR_NONE){
        return -err;
    }
    tof->Id=0;
    tof->Present=1;
    return 0;
}


int tof_vl53l0x_setup_single(VL53L0X_DEV tof,RangingConfig_e rangingConfig)
{
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

 
    status=VL53L0X_StaticInit(tof);
    if( status ){
        return status;
    }

    status = VL53L0X_PerformRefCalibration(tof, &VhvSettings, &PhaseCal);
    if( status ){
        return status;
    }

    status = VL53L0X_PerformRefSpadManagement(tof, &refSpadCount, &isApertureSpads);
    if( status ){
        return status;
    }

    status = VL53L0X_SetDeviceMode(tof, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
    if( status ){
        return status;
    }

    status = VL53L0X_SetLimitCheckEnable(tof, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
    if( status ){
        return status;
    }

    status = VL53L0X_SetLimitCheckEnable(tof, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
    if( status ){
        return status;
    }
    /* Ranging configuration */
    switch(rangingConfig) {
    case LONG_RANGE:
        signalLimit = (FixPoint1616_t)(0.1*65536);
        sigmaLimit = (FixPoint1616_t)(60*65536);
        timingBudget = 33000;
        preRangeVcselPeriod = 18;
        finalRangeVcselPeriod = 14;
        break;
    case HIGH_ACCURACY:
        signalLimit = (FixPoint1616_t)(0.25*65536);
        sigmaLimit = (FixPoint1616_t)(18*65536);
        timingBudget = 200000;
        preRangeVcselPeriod = 14;
        finalRangeVcselPeriod = 10;
        break;
    case HIGH_SPEED:
        signalLimit = (FixPoint1616_t)(0.25*65536);
        sigmaLimit = (FixPoint1616_t)(32*65536);
        timingBudget = 20000;
        preRangeVcselPeriod = 14;
        finalRangeVcselPeriod = 10;
        break;
    default:
        break;
    }

    status = VL53L0X_SetLimitCheckValue(tof,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
    if( status ){
        return status;
    }

    status = VL53L0X_SetLimitCheckValue(tof,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
    if( status ){
        return status;
    }

    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(tof,  timingBudget);
    if( status ){
        return status;
    }

    status = VL53L0X_SetVcselPulsePeriod(tof,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
    if( status ){
        return status;
    }

    status = VL53L0X_SetVcselPulsePeriod(tof,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
    if( status ){
        return status;
    }

    status = VL53L0X_SetRefCalibration(tof, VhvSettings, PhaseCal);
    if( status ){
        return status;
    }

     status = VL53L0X_SetReferenceSpads(tof, refSpadCount, isApertureSpads);
    if( status ){
        return status;
    }

    tof->LeakyFirst=1;
    return 0;
}

static void store_distance(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange)
{
    const int LeakyFactorFix8 = (int)( 0.6 *256);
    if( pRange->RangeStatus == 0 ){
        if( pDev->LeakyFirst ){
            pDev->LeakyFirst = 0;
            pDev->LeakyRange = pRange->RangeMilliMeter;
        }else{
            pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
        }
    }else{
        pDev->LeakyFirst = 1;
    }
}
int tof_single_get_distance(VL53L0X_DEV tof)
{
	VL53L0X_Error status;
    VL53L0X_RangingMeasurementData_t data;

	status = VL53L0X_PerformSingleRangingMeasurement(tof, &data);
	if(status !=VL53L0X_ERROR_NONE){
        return status;
    }
   
   store_distance(tof,&data);
	return data.RangeMilliMeter;
}