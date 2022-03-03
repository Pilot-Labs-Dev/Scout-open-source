#ifndef __ROLLER_EYE_SYSTEM_DEFINE_H__
#define __ROLLER_EYE_SYSTEM_DEFINE_H__

#ifdef APP_ARCH_X86
#define MOTION_DETECT_DEBUG
#endif


#define ROLLER_EYE_SOCKPROXY_BASE  "/opt/sockproxy/"

#define ROLLER_EYE_FILE_HOME_PATH   "/var/roller_eye/"  //if modify this path,search "/var/roller_eye" to confirm 
#define ROLLER_EYE_DINAMIC_FILE_HOME_PATH  "/userdata/roller_eye/"


#define ROLLER_EYE_CONFIG_BASE  ROLLER_EYE_FILE_HOME_PATH"/config/"
#define ROLLER_EYE_SN_LENGTH        12
#define ROLLER_EYE_KEY_LENGTH        12
#define ROLLER_EYE_HWVER_LENGTH        10

#define SENSOR_CONFIG_PATH          ROLLER_EYE_DINAMIC_FILE_HOME_PATH"/sensor/"
#define GYRO_CALIBRATION_FILE       SENSOR_CONFIG_PATH"gyro_calibration"
#define ACC_CALIBRATION_FILE           SENSOR_CONFIG_PATH"acc_calibration"
#define MAG_CALIBRATION_FILE           SENSOR_CONFIG_PATH"mag_calibration"
#define CMD_PREFIX                                      "sudo "

#define SCRATCH_SCRIPTS_PATH                    ROLLER_EYE_DINAMIC_FILE_HOME_PATH"/scratch/scripts/"
#define NAVIGATE_PATH_PATH                    ROLLER_EYE_DINAMIC_FILE_HOME_PATH"navigate/"

#define CESPATROLL_PATH_PATH                    ROLLER_EYE_DINAMIC_FILE_HOME_PATH"/cespatroll/"

#define TIMER_TASK_PATH                                 ROLLER_EYE_DINAMIC_FILE_HOME_PATH"/timer_task/"

#define OTA_ROOT_PATH                                     ROLLER_EYE_DINAMIC_FILE_HOME_PATH"/ota/" 
#define OTA_DL_CACHE                                 OTA_ROOT_PATH"/dl/"

#define LOG_LEVEL_RELEASE_DEFAULT    ros::console::levels::Info 
#define LOG_LEVEL_DEBUG_DEFAULT       ros::console::levels::Debug
#ifdef NDEBUG
#define APP_NODE_DEBUG_LEVEL                   LOG_LEVEL_RELEASE_DEFAULT
#define CLOUD_NODE_DEBUG_LEVEL             LOG_LEVEL_RELEASE_DEFAULT
#define CORE_NODE_DEBUG_LEVEL                 LOG_LEVEL_RELEASE_DEFAULT
#define RECORD_NODE_DEBUG_LEVEL                    LOG_LEVEL_RELEASE_DEFAULT
#define RTMP_NODE_DEBUG_LEVEL                    LOG_LEVEL_RELEASE_DEFAULT
#define SENSOR_NODE_DEBUG_LEVEL                    LOG_LEVEL_RELEASE_DEFAULT
#define MOTOR_NODE_DEBUG_LEVEL                    LOG_LEVEL_RELEASE_DEFAULT
#define UI_NODE_DEBUG_LEVEL                    LOG_LEVEL_RELEASE_DEFAULT
#define WIFI_NODE_DEBUG_LEVEL                    LOG_LEVEL_RELEASE_DEFAULT
#define NAV_NODE_DEBUG_LEVEL                    LOG_LEVEL_RELEASE_DEFAULT
#define SCHED_NODE_DEBUG_LEVEL                    LOG_LEVEL_RELEASE_DEFAULT
#define UPGRADER_NODE_DEBUG_LEVEL                 LOG_LEVEL_DEBUG_DEFAULT
#define UTIL_NODE_DEBUG_LEVEL                     LOG_LEVEL_DEBUG_DEFAULT
#define PRO_TEST_NODE_DEBUG_LEVEL                 LOG_LEVEL_DEBUG_DEFAULT
#else
#define APP_NODE_DEBUG_LEVEL                   LOG_LEVEL_DEBUG_DEFAULT
#define CLOUD_NODE_DEBUG_LEVEL             LOG_LEVEL_DEBUG_DEFAULT
#define CORE_NODE_DEBUG_LEVEL                 LOG_LEVEL_DEBUG_DEFAULT
#define RECORD_NODE_DEBUG_LEVEL                    LOG_LEVEL_DEBUG_DEFAULT
#define RTMP_NODE_DEBUG_LEVEL                    LOG_LEVEL_DEBUG_DEFAULT
#define SENSOR_NODE_DEBUG_LEVEL                    LOG_LEVEL_DEBUG_DEFAULT
#define MOTOR_NODE_DEBUG_LEVEL                    LOG_LEVEL_DEBUG_DEFAULT
#define UI_NODE_DEBUG_LEVEL                    LOG_LEVEL_DEBUG_DEFAULT
#define WIFI_NODE_DEBUG_LEVEL                    LOG_LEVEL_DEBUG_DEFAULT
#define NAV_NODE_DEBUG_LEVEL                    LOG_LEVEL_DEBUG_DEFAULT
#define SCHED_NODE_DEBUG_LEVEL                    LOG_LEVEL_DEBUG_DEFAULT
#define UPGRADER_NODE_DEBUG_LEVEL                 LOG_LEVEL_DEBUG_DEFAULT
#define UTIL_NODE_DEBUG_LEVEL                     LOG_LEVEL_DEBUG_DEFAULT
#define PRO_TEST_NODE_DEBUG_LEVEL                 LOG_LEVEL_DEBUG_DEFAULT
#endif

#define SLAM_USE_KERNEL_STAMP
//#define SLAM_PRINTF_TIME_DIFF

//#define SENSOR_IMU_USE_CALIBRA

#define TIMER_TASK_TYPE_PATROL                      "patrol"

#define PARAM_SYSTEM_MONITOR_GROUP       "monitor"
#define PARAM_SYSTEM_VIDEO_GROUP       "video"
#define PARAM_SYSTEM_TZ_GROUP              "timeZone"
#define PARAM_SYSTEM_SOUND_EFFECT              "soundEffect"
#define PARAM_SYSTEM_MOTION              "motion"

//node path
#define PARAM_NODE_NAME                                      "ParamNode"
#define PARAM_VIDEO_PATH                                        PARAM_NODE_NAME"/" PARAM_SYSTEM_VIDEO_GROUP
#define PARAM_VIEDO_SAVE_PATH                           ROLLER_EYE_CONFIG_BASE"/" PARAM_SYSTEM_VIDEO_GROUP".yaml"

enum SPEAKER_CMD{
    SPEAKER_START = 0,
    SPEAKER_STOP,
};

typedef enum WIFI_OP_TYPE{
    WIFI_OP_NONE=0,
    WIFI_OP_CONFIG,
    WIFI_OP_SWITCH
}WIFI_OP_T;

#define ALIGN_IMG_WIDTH                         1280
#define ALIGN_IMG_HEIGHT                        720

#define WIFI_CONFIG_START       0          
#define WIFI_CONFIG_SUCCESS  1   
#define WIFI_CONFIG_FAILURE    2  
#define WIFI_CONFIG_CANCEL    3  

#define LOW_BATTERY_PER            20

#define MAX_VALID_TOF_DIST       2
#define TOF_AVG_COUNT                 2

#endif