#ifndef __VIDEO_STREAM_H_
#define __VIDEO_STREAM_H_
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>


#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/times.h>
#include <time.h>

#define INDEX(x) (int)log2(x)
#define HEIGHT_INDEX 1
#define WIDTH_INDEX 0  


#if 0
/* Data types */
typedef char char_t;
typedef unsigned char uchar_t;
typedef int int_t;
typedef unsigned int uint_t;
typedef short short_t;
typedef unsigned short ushort_t;
typedef float float_t;
typedef unsigned long ulong_t;
typedef unsigned long long ulonglong_t;
typedef long long_t;
typedef int Status_t;
typedef unsigned long long ulonglong_t;
#endif
//#if 0
/*configuration structure definitions*/
/* Sensor mic audio profile */
/* Sensor night vision */
typedef enum {
	PLUGIN_CMD_START=0,
	H264_PLUGIN=1,
	FLV_PLUGIN=2,
	SNAPSHOT_PLUGIN=4,
	AAC_PLUGIN=8,
	MOTION_DETECT_PLUGIN=16,
	RTMP_PLUGIN=32,
	PLUGIN_CMD_END=63,
}PLUGIN_STATUS_T;


#if 0
typedef struct thread_info{
	int_t thread_type;
	pthread_t thread_id;
	int_t thread_status;
}THREAD_INFO_T,*THREAD_INFO_Tp;

typedef enum thread_status_types{
	THREAD_RUNNING;
	THREAD_RESTART;
}
//#endif	
typedef struct audio_profile {
	int_t sensitivity;
	uint_t bit_rate; // Refer AUDIO_BIT_RATE_T
	uint_t sampling_rate; // Refer AUDIO_SAMPLING_RATE_T
	uint_t codec; // Refer AUDIO_CODEC_T
	uint_t volume; // Refer SCALED_LEVEL_T
	uint_t audio_res_flag; 
}AUDIO_PROFILE_T, *AUDIO_PROFILE_Tp;
#endif
#if 0
/* Sensor mic attributes */
enum SensorAttrsMic {
	SEN_MIC_ATTR_START, // Do not remove
	SEN_MIC_ATTR_OUTPUT = 1, // Can be in voltage or decibel
	SEN_MIC_ATTR_SENSITIVITY = 2,
	SEN_MIC_ATTR_BIT_RATE = 4,
	SEN_MIC_ATTR_SAMPLING_RATE = 8,
	SEN_MIC_ATTR_AUDIO_CODECS = 16,
	SEN_MIC_ATTR_VOLUME = 32,

	SEN_MIC_ATTR_END = 64 // Do not remove, change the value on addition of a new attribute
} SENSOR_ATTRS_MIC_T;
//#endif
/* Sensor camera video profile */
typedef struct video_profile {
	uint_t bit_rate;    // Refer VIDEO_BIT_RATE_T
	uint_t fps; // Refer FPS_T
	uint_t resolution; // Refer RESOLUTION_T
	uint_t codec; // Refer VIDEO_CODEC_T
	uint_t brightnes; // Refer SCALED_LEVEL_T
	uint_t contrast; // Refer SCALED_LEVEL_T
	int_t record_state; // Refer OPER_STATE_T
	int_t record_duration; //In seconds
	char_t pixel_count[MAX_PIXEL_COUNT_LEN];
	char_t video_sensor_type[MAX_VIDEO_SENSOR_TYPE_LEN]; // like CMOS
	char_t video_sensor_size[MAX_VIDEO_SENSOR_SIZE_LEN]; // like APS-C
	int_t night_vision_oper_state; // Refer OPER_STATE_T
	int_t night_vision_oper_mode;// Refer MODE_T
	uint_t ptz_motor_direction; // Refer MOTOR_DIRECTION_T
	uint_t video_res_flag;
} VIDEO_PROFILE_T, *VIDEO_PROFILE_Tp;

#if 0
/* Sensor camera attributes */
enum SensorAttrsCamera {
	SEN_CAMERA_ATTR_START, // Do not remove
	SEN_CAMERA_ATTR_BIT_RATE = 1,
	SEN_CAMERA_ATTR_FPS = 2,
	SEN_CAMERA_ATTR_RESOLUTION = 4,
	SEN_CAMERA_ATTR_CODEC = 8,
	SEN_CAMERA_ATTR_BRIGHTNESS = 16,
	SEN_CAMERA_ATTR_CONTRAST = 32,
	SEN_CAMERA_ATTR_RECORD_STATE = 64,
	SEN_CAMERA_ATTR_RECORD_DURATION = 128,
	SEN_CAMERA_ATTR_STREAM_START = 256,
	SEN_CAMERA_ATTR_STOP_STREAM = 512,
	SEN_CAMERA_ATTR_NIGHT_VISION_OPER_STATE = 1024,
	SEN_CAMERA_ATTR_NIGHT_VISION_OPER_MODE = 2048,
	SEN_CAMERA_ATTR_PTZ = 4096,
	SEN_CAMERA_ATTR_END = 8192 // Do not remove, change the value on addition of a new attribute 
} SENSOR_ATTRS_CAMERA_T;
#endif
#endif
/*media buffer */
#define MEDIA_BUFFER_SIZE 50000
#define VIDEO_BUFFER_COUNT 20
#define AUDIO_BUFFER_COUNT 20
#define RTMP_BUFFER_COUNT  15
typedef struct buffer_data{
	int type;
	uchar_t buffer[MEDIA_BUFFER_SIZE];
	int len;
	int updated_time;
}MEDIA_BUFFER_T;

typedef enum {
	VIDEO_TYPE=5,
	AUDIO_TYPE
}BUFFER_TYPE_T,*BUFFER_TYPE_Tp;


/* Video Stream error codes */
#define SUCCESS 0
#define FAILURE -1

enum Video_stream_error_codes {
	VS_ERR_CODES_START,    //Do not remove
	VS_H264_DATA_FETCH_ERROR,
	VS_FLV_DATA_FETCH_ERROR,
	VS_AAC_DATA_FETCH_ERROR,
	VS_RTMP_CONN_ERROR,
	VS_RTMP_URL_NOT_FOUND,
	VS_BRIGHTNESS_VALUE_INVALID,
	VS_ERR_CODES_END       //Do not remove
} VS_ERROR_CODES_T;


/*Audio propertie*/
#define AUDIO_SAMPLE_RATE 8000  //11025,44100,8000
#define AUDIO_BIT_RATE  64000 
#define AUDIO_QUALITY 0    //1-high
#define AUDIO_VBR 0 //1-5 for variable bit rate 0 for constant

/* Video properties*/

#define VIDEO_WIDTH                   640//1280                            // Video Encoded Frame Width
#define VIDEO_HEIGHT                  480//720                                     // Video Encoded Frame Height
#define VIDEO_FRAME_RATE              10//30                              // Video Encoded Frame Rate
#define VIDEO_IMG_DEV_NAME    "/dev/video0"           // ISP Device Node
#define VIDEO_CAP_DEV_NAME    "/dev/video1"           // Video Codec Device Node for M2M
#define VIDEO_FRAME_NUM               120                             // Video Encoded Frame Number
#define VIDEO_SCALE                   1                                       // 1: 1, 2: 1/2, 4: 1/4 scaling down
#define VIDEO_QP                              29  
#define VIDEO_FORMAT                  V4L2_PIX_FMT_H264   //V4L2_PIX_FMT_SNX420 //V4L2_PIX_FMT_H264 //V4L2_PIX_FMT_MJPEG

/*RTMP params*/
/* RTMP meta data params*/
#define AAC_ADTS_HEADER_SIZE 7
#define FLV_TAG_HEAD_LEN 11
#define FLV_PRE_TAG_LEN 4
#define RTMP_STREAM_PROPERTY_PUBLIC      0x00000001
#define RTMP_STREAM_PROPERTY_ALARM       0x00000002
#define RTMP_STREAM_PROPERTY_RECORD      0x00000004
#define RTMP_URL "rtmp://192.168.0.10:1935/rajin/myStream "
//#define RTMP_URL "rtmp://e9bc94.entrypoint.cloud.wowza.com/app-9bdb/08d0c7c2" //"rtmp://192.168.0.10:1935/rajin/myStream "

/*motion detection params*/
/*video*/
#define MOTION_CLIP_LENGTH 10 //in secs
#define AUDIO_CLIP_LENGTH  10 //in sec
typedef enum{
	CAM_NO_MOTION,
	CAM_MOTION_DETECTED,
	CAM_MOTION_PROCESS
}MOTION_DETECT_FLAG_T;

/* Operation state */
enum OperState {
        OPER_STATE_START, // Do not remove
        DISABLED,
        ENABLED,
        OPER_STATE_END // Do not remove
} OPER_STATE_T;



#if 0
/*snap shot*/

#define MAX_PIXEL_COUNT_LEN         32
#define MAX_VIDEO_SENSOR_TYPE_LEN   32
#define MAX_VIDEO_SENSOR_SIZE_LEN   32
#define STREAM_URI_SIZE 50
typedef struct video_profile {
    int_t  conn_state;
    int_t  oper_state;
    uint_t bit_rate;    // Refer VIDEO_BIT_RATE_T
    uint_t fps; // Refer FPS_T
    uint_t resolution; // Refer RESOLUTION_T
    uint_t codec; // Refer VIDEO_CODEC_T
    uint_t brightnes; // Refer SCALED_LEVEL_T
    uint_t contrast; // Refer SCALED_LEVEL_T
    int_t record_state; // Refer OPER_STATE_T
    int_t record_duration; //In seconds
    char_t pixel_count[MAX_PIXEL_COUNT_LEN];
    char_t video_sensor_type[MAX_VIDEO_SENSOR_TYPE_LEN]; // like CMOS
    char_t video_sensor_size[MAX_VIDEO_SENSOR_SIZE_LEN]; // like APS-C
    int_t night_vision_oper_state; // Refer OPER_STATE_T
    int_t night_vision_oper_mode;// Refer MODE_T
    uint_t ptz_motor_direction; // Refer MOTOR_DIRECTION_T
    uint_t ptz_motor_layer; // Refer MOTOR_LAYER_T
    //SENSOR_NIGHT_VISION_T night_vision;
    //SENSOR_PTZ_T ptz;
    int_t ptz_oper_state; // Refer OPER_STATE_T
    uint_t motor_dance_state; //Refer OPER_STATE_T

    char_t stream_uri[STREAM_URI_SIZE];
    uint_t stream_mode;
    uint_t cooloff_time;
    uint_t video_res_flag;
} VIDEO_PROFILE_T, *VIDEO_PROFILE_Tp;

#endif
/*Sound detection params*/

#define SD_FILE_NAME "/mnt/media/Sd_1.aac"

/*function declarations*/
int jpeg_enc_yuv420(uchar_t* buffer);
void* AacEncoderThread(void*);	
void* RtmpThread(void*);
void* FlvThread(void*);
void* H264VideoThread(void *);
void* RtmpReceiveThread(void* );
void* snx_md_thread(void *);
void* SnapShotThread(void *);
int snx_isp_filter_contrast_set(int );
int snx_isp_filter_brightness_set(int);

#endif
