#include <iostream>
#include <cstdlib>
#include <thread>             // std::thread
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>
#include <linux/input.h>

#include "zlog.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "roller_eye/ros_tools.h"
#include "roller_eye/motor.h"
#include "roller_eye/plt_assert.h"
#include <iostream>
#include <fstream>

#include "roller_eye/camera_handle.hpp"
#include "roller_eye/algo_utils.h"
#include "roller_eye/status_publisher.h"
#include "roller_eye/cv_img_stream.h"
#include "roller_eye/SensorBase.h"
#include "roller_eye/plt_tools.h"
#include "roller_eye/plt_config.h"
#include "roller_eye/wifi_ops.h"
#include "sensor_msgs/Illuminance.h"
#include "roller_eye/cv_img_stream.h"
#include "std_msgs/Int32.h"
#include "roller_eye/track_trace.h"
#include "roller_eye/recorder_mgr.h"
#include <roller_eye/wifi_ops.h>
#include "roller_eye/sound_effects_mgr.h"

#include "roller_eye/start_bist.h"
#include "roller_eye/get_bist_result.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>


using namespace roller_eye;
#define DETECT_ROLL_SPEED               (1.5)

#define BIST_BOOT_ERROR				1
#define BIST_WFI_ERROR				2
#define BIST_KEY_PRESSING_ERROR		3
#define BIST_LIGHT_SENSOR_ERROR		4
#define BIST_SOUND_ERROR			5
#define BIST_TOF_ERROR				6
#define BIST_IMU_ERROR				7
#define BIST_MOTOR_WHEELS_ERROR		8
#define BIST_CAMERA_RESOLUTION_ERROR	9
#define BIST_CAMERA_DISTORTION_ERROR	10
#define BIST_BATTERY_ERROR			11
#define BIST_IRLIGHT_ERROR			12
#define BIST_BATTERY_WIFIANTENNA_ERROR 13
#define BIST_MEMERY_ERROR 14
#define BIST_DISK_ERROR 15


#define RESET_KEY_LONG_PRESS_TIME   (500)
#define WIFI_KEY_LONG_PRESS_TIME    (500)
#define POWER_KEY_LONGPRESS_TIME   (500)

#define BIST_DETECT_W         640
#define BIST_DETECT_H          480
#define BIST_DETECT 	"/var/roller_eye/devAudio/got_new_instruction/got_new_instrction.wav"
#define NAV_OUT_SPEED 0.4
#define BIST_MOVE_DIST 0.5

#define CHESSBAORD_ROW                      5
#define CHESSBAORD_COL                        5
#define CHESSBAORD_SIZE                       0.0065


#define CAM_K_FX                                        720.5631606
#define CAM_K_FY                                        723.6323653
#define CAM_K_CX                                        659.39411452
#define CAM_K_CY                                        369.00731648
#define DISTRO_K1                                       -0.36906219
#define DISTRO_K2                                       0.16914887
#define DISTRO_P1                                       -0.00147033
#define DISTRO_P2                                       -0.00345135

#define CAR_LENGTH                                   (0.1)
#define ALIGN_DISTANCE                          0.16

#define MEM_SIZE 900000
#define DISK_SIZE 7.0   //GB
class CompGreater
{
public:
    bool operator ()(const WiFiInFo& info1, const WiFiInFo& info2)
    {
        return info1.signal > info2.signal;
    }
};


class BistNode{
    public:
    BistNode();
    ~BistNode();

private:
	void MotorWheelCB(const detectConstPtr& obj);
	void tofDataCB(const sensor_msgs::RangeConstPtr &r);
	void odomRelativeCB(const nav_msgs::Odometry::ConstPtr& msg);
	void BatteryStatusCB(const statusConstPtr &s);
	void IlluminanceCB(const sensor_msgs::IlluminanceConstPtr& ptr);
	void IRLightCB(const sensor_msgs::IlluminanceConstPtr& ptr);
	void imuCB(const sensor_msgs::Imu::ConstPtr& msg);
	void imuForMotorCB(const sensor_msgs::Imu::ConstPtr& msg);
	void VideoDataCB(roller_eye::frameConstPtr frame);

	void calDistanceAndAngle(Eigen::Quaterniond& q,Eigen::Vector3d& pos,Eigen::Vector3d &goal,double &angle,double &distance);
    void LEDControl(int id,int flag)
    {
        static const char* ids[]={"gpio02_","gpio03_","gpio04_","gpio05_"};
        static const char* flags[]={"0", "1"};
        char buff[16];

        //PLOG_INFO("bist","LEDControl %d",id);
        snprintf(buff,sizeof(buff),"%s%s\n",ids[id],flags[flag]);
        ofstream of(LED_PATH);
        if(!of){
            PLOG_ERROR("bist","can't not open %s\n",LED_PATH.c_str());
            return;
        }
        of<<buff<<std::flush;
    }
    void logInit(){
		log_t arg = {
			confpath:	"/var/roller_eye/config/log/bist.cfg",
			levelpath:	"/var/roller_eye/config/log/bist.level",
			logpath:	"/var/log/node/bistNode.log",
			cname:		"bist"
		};
		int ret = dzlogInit(&arg,2);
		system("echo 20 > /var/roller_eye/config/log/bist.level");
		printf("dzlogInit:%d\r\n",ret);
		zlog_category_t *zc = zlog_get_category("bist");
		zlog_level_switch(zc,20);
    }
    void LEDProcess(int flag);
	void LEDStatus(int flag);
	void LEDAllON();
	void LEDAllOFF();
	void IrLightOn();
	void IrLightOff();
	void PCMAnalysis(int type = 0);
	void PCMgeneration();
	float ImagArticulation(const cv::Mat &image);
	int pcm2wave(const char *pcmpath, int channels, int sample_rate, const char *wavepath);
	void BistSaveImage();
	void JpgCallback(roller_eye::frameConstPtr frame);
	void getPose(float &lastX,float &lastY,float &angle);
    bool translation(float x, float speed, float xError);
	void doAlign();
	void moveByObj(float angle);
	//void GreyImagCallback(const sensor_msgs::ImageConstPtr& msg);
	void BistVideoDetect();
	void BistTofDetect();
	void BistSpeakerMicrophoneDetect();
	void BistKeyDetect();
	void BistWIFIDetect();
	void BistBatteryDetect();
	void BistLightDetect();
	void BistIMUDetect();
	void BistMotorWheelsDetect();
	void BistIRLightDetect();
	void BistWIFIAntennaDetect();
	void BistMemDetect();
	void BistDiskDetect();
	void BistDriveMotor();

	void BistPCBIMUDetect();
	void BistPCBTofDetect();
	void BistMicrophoneDetect();
	void BistPCBVideoDetect();

	void BistOpenSpaceDetect();
	AlgoUtils mAlgo;
	float mRange = 0.0;
	float mPCBRange = 0.0;
	double mDistance = 0.0;
	double mAngle = 0.0;
	int mCharging = -1;
	double mArticulation = 0.0;
	//mutex mBistMutex;
	std::mutex mtx;
	std::condition_variable cv;
	const string LED_PATH="/proc/driver/gpioctl";
	const string LOG_CFG="/var/roller_eye/config/log.conf";
	const string LOG_LEVEL="/var/roller_eye/config/log.level";
	const string PCMFILE="/tmp/test.pcm";
	const string PCM2WAVFILE="/tmp/pcm2wav.wav";
	const string WAVFILE="/tmp/test.wav";
	const string BistImage="/userdata/Bist.jpg";

	int mPcmHzs[3] = {0};
	bool mIllumMin = false;
	bool mIllumMax = false;
	//ir light off
	int mIRLightOffValu = 0;
	int mIRLightOffCount = 0;
	bool mIrLightOff = false;
	//ir light on
	int mIRLightOnValu = 0;
	int mIRLightOnCount = 0;
	bool mIrLightOn = false;

	//key
	int mWiFiFD;
	int mResetFD;
	int mPowerFD;
	int IllumCount;

    const string WIFI_KEY_TYPE="rk29-keypad";
    const string RESET_KEY_TYPE = "adc-keys";
    const string POWER_KEY_TYPE = "rk8xx_pwrkey";
    struct timeval mLastResetTime;
    struct timeval mLastWifiTime;
	struct timeval mLastPowerTime;
	bool mWiFiKey = false;
	bool mResetKey = false;
	bool mPowerKey = false;
	void processResetKey();
	void processWiFiKey();
	void processPowerKey();
	void KeyStatusLoop();
	void BistLoop();
	void BistReset();
	void BistStatus();
	void scanWiFilist(vector<WiFiInFo>& wifis );
	void scanWiFiQuality(vector<WiFiInFo>& wifis );
	void BistGreyImage();
	void BistMatImage();
	void JpgMatCb(roller_eye::frameConstPtr frame);
	bool start(roller_eye::start_bistRequest &req, roller_eye::start_bistResponse &resp);
	bool getResult(roller_eye::get_bist_resultRequest &req, roller_eye::get_bist_resultResponse &resp);
	int     mBistType;
	bool mBistStartDetct = false;
	bool mBistAllPassed = false;

	bool mBistVideoRes = false;
	bool mBistVideoDis = false;
	bool mBistVideoJpg = false;

	bool mBistTof = false;
	bool mSpeakerMicrophone = false;
	bool mKey = false;
	bool mWifi = false;
	bool mWIFIAntenna = false;
	bool mBattery = false;
	bool mLight = false;
	bool mIRLight = false;
	bool mIMU = false;
	bool mMotorWheel  = false;
	bool mMemery = false;
	bool mDisk = false;

	bool mDetectRunning = false;

	Eigen::Quaterniond mCurPose;
	Eigen::Vector3d  mCurPostion;
	//CVGreyImageStream mGreyStream;
	ros::Subscriber m_subJPG;
	ros::Subscriber m_BistJPG;

	ros::Subscriber m_subGrey;
	cv::Mat m_BistGrey;
	vector<uint8_t> mGreyData;

	ros::Subscriber mTof;
	ros::Subscriber mVideo;
	ros::Subscriber mOdomRelative;
	ros::Subscriber mBatteryStatus;
	ros::Subscriber mScrLight;
	ros::Subscriber mScrIMU;
	ros::ServiceClient mImuClient;

	shared_ptr<ros::NodeHandle> mGlobal;
	ros::Publisher  mSysEvtPub;
	ros::Publisher	mPub;

	ros::Publisher mCmdVel;
	ros::NodeHandle mGlobal2;
	ros::Subscriber mSwitch;
	ros::Publisher mCmdVel3;
	ros::ServiceServer mStartBistSrv;
	ros::ServiceServer mGetBistResSrv;
	 float mAvgTof;
	 bool mOpenSpace;
	//ros::NodeHandle mGlobal1;
};

