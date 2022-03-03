#include <iostream>
#include <cstdlib>
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
#include "std_msgs/Int32.h"
#include <roller_eye/wifi_ops.h>
#include "roller_eye/imu_calib.h"


#include "roller_eye/track_trace.h"
#include "BistNode.hpp"

#include "roller_eye/arecord.h"

static const string PROC_MOTOR_PATH="/proc/driver/motor";

using namespace roller_eye;
using namespace cv;
const char* BIST_TAG="bist";

static shared_ptr<BistNode> g_BistNode;
bool getHomeDistanceAndAngle(cv::Mat Grey,const AlgoOBjPos &objPos,float &x,float&z,float &angle);

BistNode::BistNode(){
	logInit();
	
	dzlog_info("BistNode init1.");
	mGlobal=make_shared<ros::NodeHandle>("");
	mSysEvtPub = mGlobal->advertise<std_msgs::Int32>("/system_event",1);
	mCmdVel=mGlobal->advertise<geometry_msgs::Twist>("cmd_vel",100);
	mCmdVel3=mGlobal->advertise<geometry_msgs::Twist>("cmd_vel3",100);
	
	mCurPose=Eigen::Quaterniond(1.0,0.0,0.0,0.0);
	mCurPostion=Eigen::Vector3d(0.0,0.0,0.0);

	mStartBistSrv = mGlobal->advertiseService("/BistNode/start_bist", &BistNode::start,this);
	mGetBistResSrv = mGlobal->advertiseService("/BistNode/get_bist_result", &BistNode::getResult,this);

	auto KeyStatusTh = std::thread([this]{
		KeyStatusLoop();
	});
	KeyStatusTh.detach();
	
	auto BistTh=std::thread([this](){
		BistLoop();
	});
	BistTh.detach();
}

BistNode::~BistNode(){
	dzlogfInit();
}

void BistNode::KeyStatusLoop(){	
		mWiFiFD=sensor_open_input(WIFI_KEY_TYPE.c_str());
		mResetFD = sensor_open_input(RESET_KEY_TYPE.c_str());
		mPowerFD = sensor_open_input(POWER_KEY_TYPE.c_str());
		
		time_t LastTime = time(NULL); 
		
		plt_assert(mWiFiFD>=0 && mResetFD >= 0);
		fd_set fds;
		
		//int maxFD = std::max(mWiFiFD, mResetFD) + 1;
		int maxFD = std::max(mWiFiFD, mResetFD);
		maxFD = std::max(mPowerFD, maxFD)+1;
		while(true){
			FD_ZERO(&fds);
			FD_SET(mWiFiFD,&fds);
			FD_SET(mResetFD,&fds);
			FD_SET(mPowerFD,&fds);
			if(select(maxFD,&fds,NULL,NULL,NULL)<0){
				plt_assert(errno==EINTR);
				continue;
			}
			if(FD_ISSET(mWiFiFD,&fds)){
				processWiFiKey();
			}
	
			if(FD_ISSET(mResetFD,&fds)){
			  processResetKey();
			}

			if(FD_ISSET(mPowerFD,&fds)){
			  processPowerKey();
			}
			printf ("mWiFiKey:%d,mResetKey:%d\r\n",mWiFiKey,mResetKey);
			if( (true == mWiFiKey || true == mPowerKey) && (true == mResetKey) ){
				dzlog_info("\r\n===========Bist KeyDetect.===========");
				//rmwei
				std_msgs::Int32 event;
				event.data = SYSEVT_BIST_START;
				mSysEvtPub.publish(event);
				
				system("aplay " BIST_DETECT);
				for(int index = 0;index <5;index++){
					LEDAllON();
					sleep(1);
					LEDAllOFF();
					sleep(1);
				}

				mKey = true;
				dzlog_info("Bist KeyDetect status:%s.",mKey ? "Passed" : "Failure");
				mWiFiKey = false;
				mResetKey = false;
				mBistType = 1;
				if (mPowerKey){
					mBistType = 0;
				}else{
					mBistType = 1;
				}
				mBistStartDetct = true;
				//break;
			}
			if( 10 < (time(NULL) - LastTime) ){
				mWiFiKey = false;
				mResetKey = false;
				mKey = false;
				LastTime = time(NULL);
			}
		}
		close(mWiFiFD);
		close(mResetFD);
}

bool BistNode::start(roller_eye::start_bistRequest &req, roller_eye::start_bistResponse &resp){
	if (mBistStartDetct || mDetectRunning){
		return false;
	}
	BistReset();
	mBistType = req.cmd;
	mBistStartDetct = true;
	mDetectRunning = true;
    return true;
}
bool BistNode::getResult(roller_eye::get_bist_resultRequest &req, roller_eye::get_bist_resultResponse &resp){
	if (mDetectRunning){
		return false;
	}

	printf("bist call getResult\r\n");
	map<bool, string> mResult={{true," pass"},{false," -"}};

	vector<string> vItemName={"battery","light","imu","motor","camera","cameraD","tof","speakerM","microphone","wifi","wifiAn","IR"};
	vector<bool> vResult={mBattery, mLight, mIMU,mMotorWheel,mBistVideoRes,mBistVideoDis,mBistTof,
													mSpeakerMicrophone,mSpeakerMicrophone,mWifi,mWIFIAntenna,mIRLight};

	string strResult;
	for (int i=0; i<vResult.size(); i++){
		strResult+=vItemName[i]+": "+mResult[vResult[i]]+"\n";
	}
	resp.result = strResult;

	printf("bist result: %s\r\n", resp.result.c_str());
  	return true;
}

void BistNode::BistLoop(){
	while(true){
		if(mBistStartDetct){
			mBistStartDetct = false;
			printf("start BistLoop.\r\n");
			BistReset();
			if (1==mBistType){
				BistBatteryDetect(); //ok 2
				BistIMUDetect(); //ok 6
				BistMotorWheelsDetect();
				BistTofDetect(); // 
				BistVideoDetect();
				BistWIFIAntennaDetect();
				BistWIFIDetect();	//ok 3
				BistSpeakerMicrophoneDetect();//ok 4
				BistLightDetect(); //ok 5
				BistIRLightDetect(); //ok 5
				BistMemDetect();
				BistDiskDetect();
				mDetectRunning = false;
				BistStatus();
			}else{
				//mLight = true;
				mIRLight = true;
				//mSpeakerMicrophone = true;
				//mIMU = true;
				//mBistTof = true;
				BistMemDetect();
				BistDiskDetect();
				BistDriveMotor();
				BistBatteryDetect(); //ok 2
				BistPCBVideoDetect();
				BistWIFIAntennaDetect();
				BistWIFIDetect();	//ok 3
				BistLightDetect(); //ok 5
				BistPCBIMUDetect();
				BistPCBTofDetect();
				BistMicrophoneDetect();
				mIRLight = true;
				mBistVideoDis = true;
				mDetectRunning = false;
				BistStatus();
			}
		}
		sleep(1);
	}
}
void BistNode::BistReset(){
	mBistStartDetct = false;
	mBistVideoRes = false;
	mBistVideoDis = false;
	mBistTof = false;
	mSpeakerMicrophone = false;
	mKey = false;
	mWifi = false;
	mWIFIAntenna = false;
	mBattery = false;
	mLight = false;
	mIMU = false;
	mMotorWheel = false;
	IllumCount = 0;
	mBistAllPassed = false;
	mIRLight = false;
	mMemery = false;
	mDisk = false;	
}

void BistNode::BistStatus(){
	while(!mBistStartDetct){
		if(mBattery &&
			mLight &&
			mIMU &&
			mMotorWheel &&
			mBistVideoRes &&
			mBistVideoDis &&
			mBistTof &&
			mSpeakerMicrophone &&
			mWifi &&
			mWIFIAntenna &&
			mMemery &&
			mDisk &&
			mIRLight ){
			if(!mBistAllPassed){
				dzlog_info("Bist %s status:all Passed.",__FUNCTION__);
				mBistAllPassed = true;
			}
			for(int index = 0;index <2;index++){
				LEDAllON();
				sleep(1);
				LEDAllOFF();
				sleep(1);
				if(mBistStartDetct)break;
			}
			
			if(mBistStartDetct)break;
		}

		if(!mBattery){
			LEDStatus(BIST_BATTERY_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mLight){
			LEDStatus(BIST_LIGHT_SENSOR_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mIMU){
			LEDStatus(BIST_IMU_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mMotorWheel){
			LEDStatus(BIST_MOTOR_WHEELS_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mBistVideoRes){
			LEDStatus(BIST_CAMERA_RESOLUTION_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mBistVideoDis){
			LEDStatus(BIST_CAMERA_DISTORTION_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mBistTof){
			LEDStatus(BIST_TOF_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mSpeakerMicrophone){
			LEDStatus(BIST_SOUND_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mWifi){
			LEDStatus(BIST_WFI_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mWIFIAntenna){
			LEDStatus(BIST_BATTERY_WIFIANTENNA_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mIRLight){
			LEDStatus(BIST_IRLIGHT_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mMemery){
			LEDStatus(BIST_MEMERY_ERROR);
			if(mBistStartDetct)break;
		}
		if(!mDisk){
			LEDStatus(BIST_DISK_ERROR);
			if(mBistStartDetct)break;
		}
	}
}
void BistNode::LEDAllON(){
	LEDControl(0, 1);
	LEDControl(1, 1);
	LEDControl(2, 1);
	LEDControl(3, 1);
}

void BistNode::LEDAllOFF(){
	LEDControl(0, 0);
	LEDControl(1, 0);
	LEDControl(2, 0);
	LEDControl(3, 0);
}


void BistNode::LEDProcess(int flag){
	int led0 = flag>>0&0x01;
	int led1 = flag>>1&0x01;
	int led2 = flag>>2&0x01;
	int led3 = flag>>3&0x01;
	LEDControl(0, led0);
	LEDControl(1, led1);
	LEDControl(2, led2);
	LEDControl(3, led3);
}


void BistNode::LEDStatus(int flag){
	int led0 = flag>>0&0x01;
	int led1 = flag>>1&0x01;
	int led2 = flag>>2&0x01;
	int led3 = flag>>3&0x01;

	printf("LEDStatus:%d%d%d%d\r\n",led3,led2,led1,led0);
	for(int index =0;index <2;index++){
		LEDControl(0, led0);
		LEDControl(1, led1);
		LEDControl(2, led2);
		LEDControl(3, led3);
		sleep(1);
		LEDAllOFF();
		sleep(1);
	}
	LEDAllOFF();
}

void BistNode::getPose(float &lastX,float &lastY,float &angle)
{
	lastX = 0;
	lastY = 0;
	angle = 0;
	cv::Mat img;
	m_BistGrey = img;
	bool bChess = false;
	int grepeat = 0;
	while(!bChess){
		BistMatImage();
		int repeat = 0;
		cv_status status = std::cv_status::timeout;
		while( std::cv_status::timeout == status ){
			printf("getPose.\r\n");
			std::unique_lock<std::mutex> lck(mtx);
			status = cv.wait_for(lck,std::chrono::seconds(5));
			if( (std::cv_status::timeout != status) || (5 < repeat++) ){
				break;
			}
		}
		if(!m_BistGrey.empty()){
			static const string HOME="home";
			
			AlgoOBjPos pos;
		
			int ret = mAlgo.waitObj(HOME, 3000, pos);
			if(0 == ret){
				std::cout << "left0: " << pos.left << ", " << pos.top << ", " << pos.right
						<<", " << pos.bottom <<", "<<pos.width<<", "<<pos.height<<std::endl;
				bChess = getHomeDistanceAndAngle(m_BistGrey,pos,lastX,lastY,angle);
			}

			//}
			lastY -= ALIGN_DISTANCE;
			printf("x:%f,y:%f,angle:%f\r\n",lastX,lastY,angle);
			cv::Rect2f rect(pos.left, pos.top, 
						pos.right-pos.left, pos.bottom-pos.top);
			dzlog_debug("lastX:%f,lastZ:%f,angle:%f\r\n", lastX,lastY,angle);
		}
		if( 3 < grepeat++ ){
				break;
		}
	}
}


void BistNode::BistMotorWheelsDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	
	//m_BistJPG = mGlobal->subscribe("CoreNode/jpg", 100, &BistNode::VideoDataCB, this);
	//CVGreyImageStream CVStream(1920,1080);
	//mOdomRelative=mGlobal->subscribe("MotorNode/baselink_odom_relative", 100, &BistNode::odomRelativeCB,this);
	double xDistance = 0;; 
	double yDistance = BIST_MOVE_DIST; 
	double speed = 0.2*MACC_MAX_SPEED_Y;
	//PLOG_DEBUG(UTIL_NODE_TAG, "algoMove, xDistance:%.2fm yDistance:%.2fm speed:%.2fm/s", req.xDistance, req.yDistance, req.speed);
 	Eigen::Vector3d  startPostion = mCurPostion;
	bool bFs8003Motor = false;
	string hwVer;
	PltConfig::getInstance()->getHwVersion(hwVer);
	if (hwVer.length() ==10 && hwVer[5] == '0' && hwVer[6] == '1' ){
		bFs8003Motor = true;
	}

	int ret = mAlgo.move(0, BIST_MOVE_DIST, 0.15);
	
	mAlgo.roll(M_PI,DETECT_ROLL_SPEED);
	PLOG_DEBUG(UTIL_NODE_TAG, "algoMove ret:%d", ret);
	//sleep(1);
	float lastX1 =  0,lastY1 =  0,angle1 =  0;
	getPose(lastX1,lastY1,angle1);
	//ret = mAlgo.move(BIST_MOVE_DIST, 0, 0.13);
	if (bFs8003Motor){
		ret = mAlgo.move(BIST_MOVE_DIST, 0, 0.1);
	}else{
	 	ret = mAlgo.move(BIST_MOVE_DIST, 0, 0.13);
	}
	PLOG_DEBUG(UTIL_NODE_TAG, "algoMove ret:%d", ret);
	sleep(1);
	//mAlgo.roll(M_PI/4,DETECT_ROLL_SPEED);
	//float lastX2 =  0,lastY2 =  0,angle2 =  0;
	//getPose(lastX2,lastY2,angle2);
	//mAlgo.roll(-M_PI/4,DETECT_ROLL_SPEED);
	//ret = mAlgo.move(-BIST_MOVE_DIST,0, 0.13);
	if (bFs8003Motor){
		ret = mAlgo.move(-BIST_MOVE_DIST, 0, 0.1);
	}else{
		ret = mAlgo.move(-BIST_MOVE_DIST,0, 0.13);
	}
	PLOG_DEBUG(UTIL_NODE_TAG, "algoMove ret:%d", ret);
	//sleep(1);
	float lastX3 =  0,lastY3 =  0,angle3 =  0;
	getPose(lastX3,lastY3,angle3);

	//if( 0.1 > abs(lastX3) && (BIST_MOVE_DIST-0.1)<abs(lastY3) && (BIST_MOVE_DIST+0.1)>abs(lastY3) && 0.06>abs(angle3) 
	if( 0.2 > abs(lastX3) && (BIST_MOVE_DIST-0.2)<abs(lastY3) && (BIST_MOVE_DIST+0.2)>abs(lastY3) && 0.2>abs(angle3) 
		/*&& 0.05 > abs(lastY3 - lastY2)*/){
		mMotorWheel = true;
	}

	if(!mMotorWheel){
		dzlog_error("Bist %s status:%s.",__FUNCTION__,mMotorWheel ? "Passed" : "Failure");
		//LEDStatus(BIST_MOTOR_WHEELS_ERROR);
	}else{
		dzlog_info("Bist %s status:%s.",__FUNCTION__,mMotorWheel ? "Passed" : "Failure");
	}
	//PLOG_INFO(BIST_TAG, "	mBistVideo:%s", mBistVideo ? "Passed" : "Failure");

}

void BistNode::BistTofDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	//if(!mTof){
	mTof=mGlobal->subscribe("SensorNode/tof",1,&BistNode::tofDataCB,this);
	//} 
	
	int repeat = 0;
	cv_status status = std::cv_status::timeout;
	while( std::cv_status::timeout == status ){
		printf("BistTofDetect.\r\n");
		std::unique_lock<std::mutex> lck(mtx);
		status = cv.wait_for(lck,std::chrono::seconds(5));
		
		if( (std::cv_status::timeout != status) || (5 < repeat++) ){
			break;
		}
	}

	mTof.shutdown();
	
	if(!mBistTof){
		//LEDStatus(BIST_TOF_ERROR);
		dzlog_error("Bist TofDetect status:%s.",mBistTof ? "Passed" : "Failure");
	}else{
		dzlog_info("Bist TofDetect status:%s.",mBistTof ? "Passed" : "Failure");
	}
	//PLOG_INFO(BIST_TAG, "	mBistTof:%s", mBistTof ? "Passed" : "Failure");

}


void BistNode::BistSpeakerMicrophoneDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	int repeat = 0;
	while( !mSpeakerMicrophone ){
		//PCMgeneration();
		//pcm2wave(PCMFILE.c_str(), 2, 0, PCM2WAVFILE.c_str());
		
		string aply = "aplay ";
		//aply += PCM2WAVFILE;
		aply += "/var/roller_eye/devAudio/bist_test.wav ";
		aply += " -d 3 &";
		string record = "arecord -c 2 -r 44100 -f S16_LE -D 1mic_loopback --buffer-size=10240 -d 5 ";
		record += WAVFILE;
		system(aply.c_str());
		system(record.c_str());
		usleep(100*1000);
		
		PCMAnalysis(1);
		//8k
		//if( (25000 < mPcmHz)&&(mPcmHz < 35000) ){
		if( (mPcmHzs[0]<1200 &&  mPcmHzs[0]>800) &&
		      (mPcmHzs[1]<6600 &&  mPcmHzs[1]>5400) &&
			  (mPcmHzs[2]<1200 &&  mPcmHzs[2]>800)){
			mSpeakerMicrophone = true;
		}else{
			dzlog_error("Bist mSpeakerMicrophone status:%d %d %d",mPcmHzs[0], mPcmHzs[1],mPcmHzs[2]);
		}
		if( 5 < repeat++ ){
			break;
		}
	}
	
	if(!mSpeakerMicrophone){
		//LEDStatus(BIST_SOUND_ERROR);
		dzlog_error("Bist mSpeakerMicrophone status:%s.",mSpeakerMicrophone ? "Passed" : "Failure");
	}else{
		dzlog_info("Bist mSpeakerMicrophone status:%s.",mSpeakerMicrophone ? "Passed" : "Failure");
	}
}

void BistNode::BistKeyDetect()
{	
	mWiFiFD=sensor_open_input(WIFI_KEY_TYPE.c_str());
	mResetFD = sensor_open_input(RESET_KEY_TYPE.c_str());
	time_t LastTime = time(NULL); 
	
	plt_assert(mWiFiFD>=0 && mResetFD >= 0);
	fd_set fds;
	
	int maxFD = std::max(mWiFiFD, mResetFD) + 1;
    while(true) {
        FD_ZERO(&fds);
        FD_SET(mWiFiFD,&fds);
        FD_SET(mResetFD,&fds);
        if(select(maxFD,&fds,NULL,NULL,NULL)<0){
            plt_assert(errno==EINTR);
            continue;
        }
        if(FD_ISSET(mWiFiFD,&fds)){
            processWiFiKey();
        }

        if(FD_ISSET(mResetFD,&fds)){
          processResetKey();
        }
        printf ("mWiFiKey:%d,mResetKey:%d\r\n",mWiFiKey,mResetKey);
        if( (true == mWiFiKey) && (true == mResetKey) ){
			dzlog_info("Bist KeyDetect.");
			mKey = true;
			break;
        }
        if( 10 < (time(NULL) - LastTime) ){
	    	mWiFiKey = false;
	    	mResetKey = false;
	    	LastTime = time(NULL);
        }
    }
	close(mWiFiFD);
	close(mResetFD);

	if(mKey){
		LEDAllON();
		sleep(1);
		LEDAllOFF();
	}
	dzlog_info("Bist KeyDetect status:%s.",mKey ? "Passed" : "Failure");
	printf("Key test succeeded.\r\n");

}

void BistNode::processWiFiKey()
{
	struct input_event ev;
	dzlog_info("processWiFiKey.");
	if(read(mWiFiFD,&ev,sizeof(ev))!=sizeof(ev)){
		return;
	}
	if(ev.type==0){
		return;
	}
	static int nCnt = 1;
	if(ev.value==1){
		mLastWifiTime=ev.time;
	}else{
		 int t=plt_timeval_diff_ms(&ev.time,&mLastWifiTime);
		 if(t>=WIFI_KEY_LONG_PRESS_TIME){
			dzlog_info("wifi key press time,%d ms",t);
			mWiFiKey = true;
		}
	}
}


void BistNode::BistWIFIAntennaDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	int repeat = 0;
	vector<WiFiInFo> wifis;
	while( wifis.empty() ){
		wifis.clear();
		scanWiFiQuality(wifis);
		if( 5 < repeat++ ){
			break;
		}
		//sleep(1);
	}
	for(int index = 0 ;index < wifis.size() ;index++){
		printf("signal:%d\r\n",wifis[index].signal);
		if(-50 < wifis[index].signal){
			mWIFIAntenna = true;
			break;
		}
	}
	if(!mWIFIAntenna){
		dzlog_error("Bist WIFIAntennaDetect status:%s.",mWIFIAntenna ? "Passed" : "Failure");
		//LEDStatus(BIST_WFI_ERROR);
	}else{
		dzlog_info("Bist WIFIAntennaDetect status:%s.",mWIFIAntenna ? "Passed" : "Failure");
	}
}

void BistNode::scanWiFiQuality(vector<WiFiInFo>& wifis )
{
    FILE* scan = nullptr;
    char buff[128];
    WiFiInFo wifi;
    int count=0,mod;
    int err=0;

//    system(CMD_PREFIX"wl scan");

    int counter = 0;
//    bool mScanning = true;
//    while (mScanning && counter++<50){
//        usleep(100);
//    }
//    if (!mScanning){
//        return;
//    }

    string cmd = CMD_PREFIX"iwlist wlan0 scan | grep Quality | awk '{print $3}' | awk -F \":\" '{print $2}'";

    scan=popen(cmd.c_str(),"r");
    if(scan==NULL){
        PLOG_ERROR(WIFI_TAG,"Open Scan WiFi list Fail\n");
        return;     
    }

    while(fgets(buff,sizeof(buff),scan)){
        string value = buff;
        wifi.signal = atoi(value.c_str());
        wifis.push_back(wifi);
    }

    fclose(scan);
    sort(wifis.begin(),wifis.end(),CompGreater());
}



void BistNode::BistWIFIDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	int repeat = 0;
	vector<WiFiInFo> wifis;
	while( wifis.empty() ){
		wifis.clear();
		scanWiFilist(wifis);
		if( 5 < repeat++ ){
			break;
		}
		//sleep(1);
	}
    if(0 < wifis.size()){
		mWifi = true;
    }
	if(!mWifi){
		dzlog_error("Bist WIFIDetect status:%s.",mWifi ? "Passed" : "Failure");
		//LEDStatus(BIST_WFI_ERROR);
	}else{
		dzlog_info("Bist WIFIDetect status:%s.",mWifi ? "Passed" : "Failure");
	}
}

void BistNode::scanWiFilist(vector<WiFiInFo>& wifis )
{
    FILE* scan = nullptr;
    char buff[128];
    WiFiInFo wifi;
    int count=0,mod;
    int err=0;

//    system(CMD_PREFIX"wl scan");

    int counter = 0;
//    bool mScanning = true;
//    while (mScanning && counter++<50){
//        usleep(100);
//    }
//    if (!mScanning){
//        return;
//    }

    string cmd = CMD_PREFIX"iwlist wlan0 scan | grep ESSID | awk '{print $1}'";

    scan=popen(cmd.c_str(),"r");
    if(scan==NULL){
        PLOG_ERROR(WIFI_TAG,"Open Scan WiFi list Fail\n");
        return;     
    }

    while(fgets(buff,sizeof(buff),scan)){
    	//printf("buff:%s\r\n",buff);
//        mod=count++%2;
//        if(mod==0){
            string ssid=buff;
            static const string SSID="ESSID:\"";
            auto idx=ssid.find(SSID);
            err=1;
            if(idx!=string::npos){
                wifi.ssid=ssid.substr(idx+SSID.size());
                if(wifi.ssid.size()>=3){
                    wifi.ssid.erase(wifi.ssid.end()-2,wifi.ssid.end());
                    unsigned char ssid[256] = {0};
                    mid_wifi_ssid_convert_utf8(ssid, wifi.ssid.c_str(), wifi.ssid.length());
                    wifi.ssid = (char*)ssid;
                    wifis.push_back(wifi);
                    err=0;
                    //printf("ssid:%s",ssid);
                }
            }
            if(err){
                printf("bad ssid:%s\r\n",buff);
            }
//        }else{
//            if(err){
//                continue;
//            }
//            static const string RSSI="RSSI: ";
//            static const string DBM=" dBm";
//            string strRssi;
//            strRssi.assign(buff);
//            auto idx=strRssi.find(RSSI)+RSSI.size();
//            auto dbmIdx=strRssi.find(DBM);
//            if (idx != string::npos && dbmIdx!=string::npos){
//                string value = strRssi.substr(idx, dbmIdx-idx);
//                wifi.signal = atoi(value.c_str());
//                wifis.push_back(wifi);
//            }else{
//                err=1;
//                printf("bad rssi:%s",buff);
//            }
//        }
    }

    fclose(scan);
    sort(wifis.begin(),wifis.end(),CompGreater());
}

void BistNode::BistBatteryDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	//LEDProcess(BIST_BATTERY_ERROR);
	mBatteryStatus=mGlobal->subscribe("SensorNode/simple_battery_status",10,&BistNode::BatteryStatusCB,this);
	
	int repeat = 0;
	cv_status status = std::cv_status::timeout;
	while( std::cv_status::timeout == status ){
		printf("Please put scout back into the charging pile.\r\n");
		std::unique_lock<std::mutex> lck(mtx);
		status = cv.wait_for(lck,std::chrono::seconds(5));
		
		if( (std::cv_status::timeout != status) || (5 < repeat++) ){
			break;
		}
	}
	mBatteryStatus.shutdown();
	
	if(!mBattery){
		dzlog_error("Bist BatteryDetect status:%s.",mBattery ? "Passed" : "Failure");
		//LEDStatus(BIST_BATTERY_ERROR);
	}else{
		dzlog_info("Bist BatteryDetect status:%s.",mBattery ? "Passed" : "Failure");
	}
}

void BistNode::BistLightDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	mScrLight=mGlobal->subscribe("SensorNode/light",20,&BistNode::IlluminanceCB,this);

#if 1
	int repeat = 0;
	cv_status status = std::cv_status::timeout;
	while( std::cv_status::timeout == status ){
		printf("please cover the light sensor.\r\n");
		std::unique_lock<std::mutex> lck(mtx);
		status = cv.wait_for(lck,std::chrono::seconds(5));
		if( (std::cv_status::timeout != status) || (5 < repeat++) ){
			break;
		}
	}
	mScrLight.shutdown();
	
	if(!mLight){
		//LEDStatus(BIST_LIGHT_SENSOR_ERROR);
		dzlog_error("Bist LightDetect status:%s.",mLight ? "Passed" : "Failure");
	}else{
		dzlog_info("Bist LightDetect status:%s.",mLight ? "Passed" : "Failure");
	}
#endif
}

void BistNode::IrLightOn(){
	//ir light on
	printf("IrLight On.\r\n");
	system("echo 99 > /proc/driver/light");
	mIRLightOnValu = 0;
	mIRLightOnCount = 0;
	mIrLightOn = true;

	//mIRLightOffValu = 0;
	//mIRLightOffCount = 0;
	mIrLightOff = false;

}

void BistNode::IrLightOff(){
	printf("IrLight Off.\r\n");
	system("echo 0 > /proc/driver/light");
	//mIRLightOnValu = 0;
	//mIRLightOnCount = 0;
	mIrLightOn = false;

	mIRLightOffValu = 0;
	mIRLightOffCount = 0;
	mIrLightOff = true;
}


void BistNode::BistIRLightDetect(){
	dzlog_info("\r\n============%s============",__FUNCTION__);
	mScrLight=mGlobal->subscribe("SensorNode/light",20,&BistNode::IRLightCB,this);

#if 1
	mIRLightOnValu = 0;
	mIRLightOnCount = 0;
	mIrLightOn = true;

	mIRLightOffValu = 0;
	mIRLightOffCount = 0;
	mIrLightOff = false;

	IrLightOff();
	int repeat = 0;
	cv_status status = std::cv_status::timeout;
	while( std::cv_status::timeout == status ){
		printf("please cover the light sensor.\r\n");
		std::unique_lock<std::mutex> lck(mtx);
		status = cv.wait_for(lck,std::chrono::seconds(5));
		if( (std::cv_status::timeout != status) || (5 < repeat++) ){
			break;
		}
		if(mIrLightOn){
			IrLightOff();
		}else{
			IrLightOn();
		}
	}
	mScrLight.shutdown();
	IrLightOff();
	if(!mIRLight){
		//LEDStatus(BIST_IRLIGHT_ERROR);
		dzlog_error("Bist %s status:%s.",__FUNCTION__,mIRLight ? "Passed" : "Failure");
	}else{
		dzlog_info("Bist %s status:%s.",__FUNCTION__,mIRLight ? "Passed" : "Failure");
	}
#endif
}

void BistNode::BistIMUDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	
	geometry_msgs::Twist vel;
	vel.linear.y=0.1;
	mCmdVel.publish(vel);
	usleep(1);
	
	if (!mImuClient){
		mImuClient=mGlobal->serviceClient<imu_calib>("/imu_calib");
	}
	int repeat = 0;
	bool bCalibImu = false;
	while(!bCalibImu){
		if (mImuClient){
			imu_calib::Request req;
			imu_calib::Response resp;
			if (mImuClient.call(req, resp)){
				bCalibImu = true;
			}
		}
		if(5 < (repeat++) ){
			break;
		}
	}

	
	if (!bCalibImu){
		mIMU = false;
		//LEDStatus(BIST_IMU_ERROR);
		dzlog_error("Bist IMUDetect calib imu: Failure.");
		return;
	}
	mScrIMU = mGlobal->subscribe("SensorNode/imu", 1000, &BistNode::imuCB,this);
	repeat = 0;
	cv_status status = std::cv_status::timeout;
	while( std::cv_status::timeout == status ){
		std::unique_lock<std::mutex> lck(mtx);
		status = cv.wait_for(lck,std::chrono::seconds(5));
		if( (std::cv_status::timeout != status) || (5 < repeat++) ){
			break;
		}
	}
	mScrIMU.shutdown();
	
	if(!mIMU){
		//LEDStatus(BIST_IMU_ERROR);
		dzlog_error("Bist IMUDetect status:%s.",mIMU ? "Passed" : "Failure");
	}else{
		dzlog_info("Bist IMUDetect status:%s.",mIMU ? "Passed" : "Failure");
	}
}

void BistNode::BistVideoDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	m_BistJPG = mGlobal->subscribe("CoreNode/jpg", 100, &BistNode::VideoDataCB, this);

	int repeat = 0;
	cv_status status = std::cv_status::timeout;
	while( std::cv_status::timeout == status ){
		printf("Bist Video RESOLUTION Detect.\r\n");
		std::unique_lock<std::mutex> lck(mtx);
		status = cv.wait_for(lck,std::chrono::seconds(5));
		if( (std::cv_status::timeout != status) || (5 < repeat++) ){
			break;
		}
	}
	m_BistJPG.shutdown();
	if(!mBistVideoRes){
		//LEDStatus(BIST_CAMERA_RESOLUTION_ERROR);
		dzlog_error("Bist %s (%d) status:%s.",__FUNCTION__,BIST_CAMERA_RESOLUTION_ERROR,mBistVideoRes ? "Passed" : "Failure");
	}else{
		//dzlog_info("Bist %s status:%s.",__FUNCTION__,mBistVideo ? "Passed" : "Failure");
		

	//
#if 0
		mVideo=mGlobal->subscribe("CoreNode/chargingPile",1,&BistNode::MotorWheelCB,this);
		
		repeat = 0;
		status = std::cv_status::timeout;
		while( std::cv_status::timeout == status ){
			printf("Bist Video DISTORTION Detect.\r\n");
			std::unique_lock<std::mutex> lck(mtx);
			status = cv.wait_for(lck,std::chrono::seconds(5));
			
			if( (std::cv_status::timeout != status) || (5 < repeat++) ){
				break;
			}
		}
		mVideo.shutdown();
#endif
		BistSaveImage();

		repeat = 0;
		status = std::cv_status::timeout;
		while( std::cv_status::timeout == status ){
			printf("Bist Video DISTORTION Detect.\r\n");
			std::unique_lock<std::mutex> lck(mtx);
			status = cv.wait_for(lck,std::chrono::seconds(5));
			
			if( (std::cv_status::timeout != status) || (5 < repeat++) ){
				break;
			}
		}

		mArticulation = 0.0;
		if(mBistVideoJpg){
				cv::Mat videoMat = cv::imread(BistImage);
				cv::Mat videoMat1 = cv::imread("1.png");
				cv::Mat videoMat2 = cv::imread("2.png");
				
				cv::Mat res_img;
				cv::resize(videoMat, res_img, cv::Size(640,640), 0, 0, cv::INTER_NEAREST);
				printf ("%f,%f,%f\r\n", 
				ImagArticulation(videoMat1),ImagArticulation(videoMat2),ImagArticulation(res_img));
				//960*540 100*100
				//910,490,1100,590
				//printf(videoMat.ros);
				
				if(!videoMat.empty()){
		//			cv::Rect2f bbox = cv::Rect2f(910,490,100, 100);
		//			cv::Mat rioImag = videoMat(bbox);
					mArticulation = ImagArticulation(videoMat);
					dzlog_info("Bist %s Articulation:%f\r\n",__FUNCTION__, mArticulation);
					//cv::imwrite("/userdata/roi.jpg", videoMat);
				}

		}

//		if(!mBistVideoDis){
//			dzlog_error("Bist %s can't find chargingPile!!!!",__FUNCTION__);
//		}
		if(2.0 > mArticulation){
			mBistVideoDis = false;
			//LEDStatus(BIST_CAMERA_DISTORTION_ERROR);
			dzlog_error("Bist %s (%d) status:%s.",__FUNCTION__,BIST_CAMERA_DISTORTION_ERROR,"Failure");
		}else{
			mBistVideoDis = true;
			dzlog_info("Bist %s status:%s.",__FUNCTION__,"Passed");
		}
	}
}


void BistNode::BistPCBVideoDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	m_BistJPG = mGlobal->subscribe("CoreNode/jpg", 100, &BistNode::VideoDataCB, this);

	int repeat = 0;
	cv_status status = std::cv_status::timeout;
	while( std::cv_status::timeout == status ){
		printf("Bist Video RESOLUTION Detect.\r\n");
		std::unique_lock<std::mutex> lck(mtx);
		status = cv.wait_for(lck,std::chrono::seconds(5));
		if( (std::cv_status::timeout != status) || (5 < repeat++) ){
			break;
		}
	}
	m_BistJPG.shutdown();
	if(!mBistVideoRes){
		//LEDStatus(BIST_CAMERA_RESOLUTION_ERROR);
		dzlog_error("Bist %s (%d) status:%s.",__FUNCTION__,BIST_CAMERA_RESOLUTION_ERROR,mBistVideoRes ? "Passed" : "Failure");
	}else{
		//dzlog_info("Bist %s status:%s.",__FUNCTION__,mBistVideo ? "Passed" : "Failure");
	}
}

void BistNode::VideoDataCB(roller_eye::frameConstPtr frame)
{
	printf("VideoDataCB.\r\n");
    if (frame->data.size() <= 0) {
        PLOG_DEBUG(REC_TAG, "frame data is empty.");
        return;
    }
	mBistVideoRes = true;
	//lock_guard<mutex> lk(mBistMutex);
	std::unique_lock<std::mutex> lck(mtx);
	cv.notify_all();	
}

void BistNode::BistPCBIMUDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);	

	mScrIMU = mGlobal->subscribe("SensorNode/imu", 1000, &BistNode::imuCB,this);
	int repeat = 0;
	cv_status status = std::cv_status::timeout;
	while( std::cv_status::timeout == status ){
		std::unique_lock<std::mutex> lck(mtx);
		status = cv.wait_for(lck,std::chrono::seconds(5));
		if( (std::cv_status::timeout != status) || (5 < repeat++) ){
			break;
		}
	}
	mScrIMU.shutdown();
	
	if(!mIMU){
		//LEDStatus(BIST_IMU_ERROR);
		dzlog_error("Bist IMUDetect status:%s.",mIMU ? "Passed" : "Failure");
	}else{
		dzlog_info("Bist IMUDetect status:%s.",mIMU ? "Passed" : "Failure");
	}
}
void BistNode::BistPCBTofDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);

	mPCBRange = 0.0;
	mTof=mGlobal->subscribe("SensorNode/tof",1,&BistNode::tofDataCB,this);	
	
	for (int i =0; i<100;  i++){
		if (isinf(mPCBRange)  || abs(mPCBRange) > 0.0){
			mBistTof = true;
			break;
		}
		usleep(100*1000);
	}	
	mTof.shutdown();
	
	if(!mBistTof){
		//LEDStatus(BIST_TOF_ERROR);
		dzlog_error("Bist TofDetect status:%s.",mBistTof ? "Passed" : "Failure");
	}else{
		dzlog_info("Bist TofDetect status:%s.",mBistTof ? "Passed" : "Failure");
	}
	//PLOG_INFO(BIST_TAG, "	mBistTof:%s", mBistTof ? "Passed" : "Failure");

}
#define SAMPLE_RATE	22050 //16000
#define AUDIO_CHANNELS	1
#define SAMPLES_PRE_FRAME	1024
#define BYTES_PRE_FRAME		(SAMPLES_PRE_FRAME * 2 * AUDIO_CHANNELS)

void BistNode::BistMicrophoneDetect()
{
	dzlog_info("\r\n============%s============",__FUNCTION__);
	int ret = arecord_init(SND_PCM_FORMAT_S16_LE, AUDIO_CHANNELS, SAMPLE_RATE);
	if (ret !=0){
		dzlog_info("arecord_init failed!\n");
		mSpeakerMicrophone = false;
		return;
	}
    u_char buf[BYTES_PRE_FRAME]={0};
	dzlog_info("aac capture running.\n");

	int nCnt = 0;
	mSpeakerMicrophone = false;
	const int  IGNORE_CNT = 5;
	const int  SAMPLE_CNT = 20;
	do {
		ret = arecord_read(buf, BYTES_PRE_FRAME);
		if (nCnt<IGNORE_CNT){
			continue;
		}
		if(ret <=0) {
			dzlog_info("arecord_read() failed\n");
		} else {
			for (int i=0; i<ret; i++){
				if (buf[i] !=0 ){
					mSpeakerMicrophone = true;
				}
			}
			if (mSpeakerMicrophone){
				break;
			}
		}
	}while(nCnt++<SAMPLE_CNT) ;

	arecord_uninit();

	if(!mSpeakerMicrophone){
		dzlog_error("Bist mSpeakerMicrophone status:%s.",mSpeakerMicrophone ? "Passed" : "Failure");
	}else{
		dzlog_info("Bist mSpeakerMicrophone status:%s.",mSpeakerMicrophone ? "Passed" : "Failure");
	}
}

static int g_calibrate_cnt=0;
static sensor_msgs::Imu g_preIMU;
#define CALIBRATE_MAX_CNT       500
#define G_VALUE     9.7833

static Eigen::Vector3d g_gyro_offset(0,0,0);
static Eigen::Vector3d g_acc_offset(0,0,0);
static const Eigen::Vector3d G(0,0,G_VALUE);
static Eigen::Quaterniond g_pose(1,0,0,0);
static Eigen::Vector3d g_positon(0,0,0);
static Eigen::Vector3d g_vel(0,0,0);

static Eigen::Quaterniond imuToQuatenion(Eigen::Vector3d &dt)
{
    return Eigen::Quaterniond(1,dt.x()/2,dt.y()/2,dt.z()/2);
}

static bool imuCalibrate(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Vector3d accel(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    if(g_calibrate_cnt++<CALIBRATE_MAX_CNT){
        g_acc_offset+=(accel-G);
        g_gyro_offset+=gyro;
        if(g_calibrate_cnt==CALIBRATE_MAX_CNT){
            g_acc_offset/=CALIBRATE_MAX_CNT;
            g_gyro_offset/=CALIBRATE_MAX_CNT;

            std::cout<<"gyro offset:\n"<<g_gyro_offset<<std::endl;
            std::cout<<"acc offset:\n"<<g_acc_offset<<std::endl;
        }
        g_preIMU=*msg;
        return true;
    }
    return false;
}
    void cvMatToEigenRotation(cv::Mat& mat,Eigen::Matrix3d& rotation)
    {
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++){
                rotation(i,j)=mat.at<double>(i,j);
            }
    }

    bool calcCameraPoseEx(float z,cv::Mat &grey,Eigen::Vector3d& pos, cv::Rect &roi, float &xdist, float &zdist,float &angle)
    {
        cv::Size size(CHESSBAORD_COL-1,CHESSBAORD_ROW-1);
        vector<cv::Point2f> corners;
        int totalConers=size.width*size.height;
        cv::Mat roiGrey=grey(roi);
        cv::Mat clachedGrey;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(roiGrey,clachedGrey);        

    #if SAVE_IMG
        char filenName[128];  
        static int notFoundCnt = 0;
        static int foundCnt = 0;
    #endif
        PLOG_DEBUG(BACKING_UP_TAG,"cv::findChessboardCorners\n");
        if(!cv::findChessboardCorners(clachedGrey,size,corners)){
            PLOG_DEBUG(BACKING_UP_TAG,"cann't find corners!!!\n");
            #if SAVE_IMG
            sprintf(filenName, "/userdata/roller_eye/chessboard/err_%d.bmp", (notFoundCnt++)%20);
            cv::imwrite(filenName,clachedGrey);
            sprintf(filenName, "/userdata/roller_eye/chessboard/err_full_%d.bmp", (notFoundCnt++)%20);
            cv::imwrite(filenName,grey);
            #endif
            return false;
        }
        #if SAVE_IMG
        sprintf(filenName, "/userdata/roller_eye/chessboard/ok_%d.bmp", (foundCnt++)%20);
        cv::imwrite(filenName,clachedGrey);
        #endif
        if((int)corners.size()!=totalConers){
            PLOG_WARN(BACKING_UP_TAG,"corner count error!!!\n");
            return false;
        }
        
        //std::cout<<"roi:("<<roi.x<<","<<roi.y<<","<<roi.height<<","<<roi.width<<") ";
        cornerSubPix(roiGrey,corners,cv::Size(2, 2),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,30,0.1));
        for(int i=0;i<(int)corners.size();i++){
           corners[i].x+=roi.x;
           corners[i].y+=roi.y;
            // std::cout<<"("<<corners[i].x<<","<<corners[i].y<<") ";
            // if(i%(CHESSBAORD_COL-1)==CHESSBAORD_COL-2){
            //      std::cout<<std::endl;
            // }
            // if(i==(int)corners.size()-1){
            //     // std::cout<<roi<<std::endl;
            // }
        }
        vector<cv::Point3f> reals;
        float XOFFSET=(CHESSBAORD_COL-2)*CHESSBAORD_SIZE/2;
        float YOFFSET=(CHESSBAORD_ROW-2)*CHESSBAORD_SIZE/2;
        for(int i=0;i<CHESSBAORD_ROW-1;i++)
            for(int j=0;j<CHESSBAORD_COL-1;j++){
                reals.emplace_back(j*CHESSBAORD_SIZE-XOFFSET,i*CHESSBAORD_SIZE-YOFFSET,z);
            }
        
        cv::Mat camK=(cv::Mat_<double>(3,3)<<CAM_K_FX,0.0,CAM_K_CX,0.0,CAM_K_FY,CAM_K_CY,0.0,0.0,1.0);
        cv::Mat distro=(cv::Mat_<double>(4,1)<<DISTRO_K1,DISTRO_K2,DISTRO_P1,DISTRO_P2);
        cv::Mat r,t,R;
        solvePnP( reals, corners, camK, distro, r, t, false/*, cv::SOLVEPNP_EPNP */);
        cv::Rodrigues(r, R);
        //std::cout<<"r:"<<r<<std::endl<<"R:"<<R<<std::endl;
#ifdef  BACKING_UP_CALIBR
        std::cout<<"r:"<<R<<std::endl<<"t:"<<t<<std::endl;
#endif

        cv::Mat_<float> Tvec;
        t.convertTo(Tvec, CV_32F);  

        angle =  atan2(R.at<double>(1, 0), R.at<double>(0, 0));

        Eigen::Matrix3f R_n;
        Eigen::Vector3f T_n;
        cv::cv2eigen(R, R_n);
        cv::cv2eigen(Tvec, T_n);
        Eigen::Vector3f P_oc;

        P_oc = -R_n.inverse()*T_n;
        std::cout<< "world coord" << P_oc << std::endl;
        xdist = P_oc[0];
        zdist = P_oc[2];

        std::cout<<"xdist 0: "<<xdist<<" zdist 0:"<<zdist<<" angle："<<angle<<endl;
        xdist -= sin(angle)*CAR_LENGTH;
        zdist = cos(angle)*zdist;
        zdist += ALIGN_DISTANCE;
        std::cout<<"xdist 1: "<<xdist<<" zdist 1:"<<zdist<<" angle："<<angle<<endl;

        Eigen::Vector3d trans;
        Eigen::Matrix3d rotation;

        cvMatToEigenRotation(R,rotation);
        Eigen::Matrix3d rot=rotation.transpose();//R^(-1)=R^(T)
       
        trans.x()=t.at<double>(0,0);
        trans.y()=t.at<double>(1,0);
        trans.z()=t.at<double>(2,0);
        pos=rot*trans*(-1);
        return true;
    }

bool getCameraPose(cv::Mat &Grey,float z,Eigen::Vector3d& pos,cv::Rect2f& roi, float &xdist, float &zdist,float &angle)
 {
	 cv::Mat grey = Grey;
//	  if(CVStream.getCVImg(grey)<0){
//		 return false;
//	 }
	 cv::Rect rect(round(roi.x*grey.cols),round(roi.y*grey.rows),round(roi.width*grey.cols),round(roi.height*grey.rows));

	 rect.x=std::max(0,rect.x);
	 rect.x=std::min(rect.x,grey.cols-1);
	 rect.width=std::min(rect.width,grey.cols-rect.x);
	 rect.y=std::max(0,rect.y);
	 rect.y=std::min(rect.y,grey.rows-1);
	 rect.height=std::min(rect.height,grey.rows-rect.y);
	 
	 return calcCameraPoseEx(z,grey,pos,rect, xdist, zdist, angle);
 }


bool getHomeDistanceAndAngle(cv::Mat Grey,const AlgoOBjPos &objPos,float &x,float&z,float &angle)
{
	int i;
	bool bRet = false;
	Eigen::Vector3d position;
	
	//pos = 1.5pos
	float scale = 0.8;
	AlgoOBjPos pos = objPos;
	pos.left = static_cast<int> (scale*pos.left);
	pos.top = static_cast<int> (scale*pos.top);
	pos.bottom = static_cast<int> (pos.bottom/scale);
	pos.right = static_cast<int> (pos.right/scale);

	pos.bottom = pos.bottom>pos.height ? pos.height : pos.bottom;
	pos.right = pos.right>pos.width ? pos.width : pos.right;
	std::cout << "left: " << pos.left << ", " << pos.top << ", " << pos.right
				<<", " << pos.bottom <<", "<<pos.width<<", "<<pos.height<<std::endl;
	cv::Rect2f rio((float)pos.left/pos.width,(float)pos.top/pos.height,(float)(pos.right-pos.left)/pos.width,(float)(pos.bottom-pos.top)/pos.height);
	cv::Mat MatRio = Grey(rio);
	cv::imwrite("/userdata/rio.jpg", MatRio);
	for(i=0;i<3;i++){
		bRet = getCameraPose(Grey,0,position,rio, x, z, angle);
		if(bRet){
				break;
		}
		usleep(100*1000);
	}

#if 0
	if(bRet){
//		mAngle = angle;
//		mROI=rio;
//		mROIValid=true;
	}	 
#endif
	PLOG_DEBUG(BACKING_UP_TAG,"getHomeDistanceAndAngle detect mangle=%f, x=%f, z=%f, angle=%f\n",angle,x,z,angle);
	return bRet;
}

static void imuDoPoseEstimate(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(imuCalibrate(msg)){
        return;
    }
    static double rt=0;
    Eigen::Vector3d a_pre(g_preIMU.linear_acceleration.x,g_preIMU.linear_acceleration.y,g_preIMU.linear_acceleration.z);
    Eigen::Vector3d a_cur(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    Eigen::Vector3d g_pre(g_preIMU.angular_velocity.x,g_preIMU.angular_velocity.y,g_preIMU.angular_velocity.z);
    Eigen::Vector3d g_cur(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);

    a_pre-=g_acc_offset;
    a_pre=g_pose*a_pre;
    a_cur-=g_acc_offset;
    g_pre-=g_gyro_offset;
    g_cur-=g_gyro_offset;

    auto t=(msg->header.stamp-g_preIMU.header.stamp).toSec();
    Eigen::Vector3d g_mid=(g_cur+g_pre)*t/2;
    g_pose=g_pose*imuToQuatenion(g_mid);
    Eigen::Vector3d a_mid=(g_pose*a_cur+a_pre)/2-G;

    g_positon+=(g_vel*t+a_mid*t*t/2);
    g_vel+=a_mid*t;
    
    for(int i=0;i<3;i++){
        if(abs(g_vel[i])>0){
            g_vel[i]=0;
        }
    }
    rt+=t;
    if(rt>0.1){
        ROS_INFO("P (%f,%f,%f) V(%f,%f,%f),Q(%f,%f,%f,%f)",g_positon.x(),g_positon.y(),g_positon.z(),g_vel.x(),g_vel.y(),g_vel.z(),g_pose.x(),g_pose.y(),g_pose.z(),g_pose.w());
        ROS_INFO("Accel(%f,%f,%f)",a_mid.x(),a_mid.y(),a_mid.z());
        rt=0;
//        g_path.header.frame_id="world";
//        g_path.header.stamp=msg->header.stamp;
//        geometry_msgs::PoseStamped pose;
//        pose.header.frame_id="world";
//        pose.header.stamp=msg->header.stamp;
//        pose.pose.position.x=g_positon.x();
//        pose.pose.position.y=g_positon.y();
//        pose.pose.position.z=g_positon.z();
//        pose.pose.orientation.w=g_pose.w();
//        pose.pose.orientation.x=g_pose.x();
//        pose.pose.orientation.y=g_pose.y();
//        pose.pose.orientation.z=g_pose.z();
//        g_path.poses.push_back(pose);
//        g_path_pub.publish(g_path);
//        g_pose_pub.publish(pose);
    }
}

static void imuDoCalcRollAngle(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(imuCalibrate(msg)){
        return;
    }
    Eigen::Vector3d accel(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    static double rt=0;
    static Eigen::Vector3d gyroMeasure(0.0,0.0,0.0);
    double pitch,roll;

    auto t=(msg->header.stamp-g_preIMU.header.stamp).toSec();
    gyroMeasure+=(gyro-g_gyro_offset)*t;
    auto accCalibra=accel;//accel-g_acc_offset;
    pitch=atan(accCalibra.y()/accCalibra.z());
    roll=asin(-accCalibra.x()/G_VALUE);
    rt+=t;
    double RAD2DEGREE=180/M_PI;
    if(rt>0.5){
        auto gyroDegree=gyroMeasure*RAD2DEGREE;
        std::cout<<"gyro:("<<gyroDegree.x()<<","<<gyroDegree.y()<<","<<gyroDegree.z()<<")(degree)"<<std::endl;
        std::cout<<"accl:("<<pitch*RAD2DEGREE<<","<<roll*RAD2DEGREE<<") (degree)"<<std::endl;
        std::cout<<std::endl;
        rt=0;
    }

        double degree = pitch*RAD2DEGREE;  
//    if (degree<-50){
//        mAlgo.move(0,-0.02,0.4);  
//    }else if (degree< -40){
//        mAlgo.move(0,-0.02,0.4);  
//    }else if (degree< -30){
//        mAlgo.move(0,-0.02,0.3);          
//    }else if (degree< -20){
//        mAlgo.move(0,-0.02,0.2);          
//    }else if (degree< -10){
//        mAlgo.move(0,-0.02,0.2);          
//    }else if (degree< -2){
//        mAlgo.move(0,-0.02,0.2);          
//    }else if (degree< 2){        
//    }else if (degree<10){
//        mAlgo.move(0,0.02,0.2);              
//    }else if (degree<20){
//        mAlgo.move(0,0.02,0.2);               
//    }else if (degree< 30){
//        mAlgo.move(0,0.02,0.3);               
//    }else if (degree< 40){
//        mAlgo.move(0,0.02,0.4);         
//    }else{
//        mAlgo.move(0,0.02,0.3);         
//    }
}



void BistNode::imuCB(const sensor_msgs::Imu::ConstPtr& msg)
{
//	 printf("imuCB:	%lf %lf %lf %lf %lf %lf %lf\n",
//		 	msg->header.stamp.toSec(),
//		 	msg->angular_velocity.x,
//		 	msg->angular_velocity.y,
//		 	msg->angular_velocity.z,
//		 	msg->linear_acceleration.x,
//		 	msg->linear_acceleration.y,
//		 	msg->linear_acceleration.z);
		 	    
	if (1 == mBistType){
		imuDoPoseEstimate(msg);
		imuDoCalcRollAngle(msg); 
		mIMU = true;
		std::unique_lock<std::mutex> lck(mtx);
		cv.notify_all();
	}else{
		static int imuCnt = 0;
		if (imuCnt++>5){
			mIMU = true;
			std::unique_lock<std::mutex> lck(mtx);
			cv.notify_all();
		}
	}
	
}



void BistNode::IlluminanceCB(const sensor_msgs::IlluminanceConstPtr& ptr)
{
	//ROS_DEBUG("night mode Illuminance=%f",ptr->illuminance);
	//printf ("night mode Illuminance=%f\r\n",ptr->illuminance);
	int illum = static_cast<int>(ptr->illuminance);
    int mPrevCh0 = (illum>>16) & 0xFFFF;
    printf ("night mode mPrevCh0=%d,mPrevCh1:=%d\r\n",mPrevCh0,(illum) & 0xFFFF);
#if 0
    static int MaxCount = 0;
    static int MinCount = 0;
    if(40 < mPrevCh0 ){
    	if(2 < MaxCount++){
			mIllumMax = true;
    	}
    }
    if(10 > mPrevCh0 ){
	    if(2 < MinCount++){
			mIllumMin = true;
	    }
    }
    if( mIllumMax && mIllumMin ){
		mLight = true;
		std::unique_lock<std::mutex> lck(mtx);
		cv.notify_all();
    }
#endif
    if(10 < mPrevCh0){
    	IllumCount++;
    }
    if(5 < IllumCount){
		mLight = true;
		std::unique_lock<std::mutex> lck(mtx);
		cv.notify_all();
    }
}

void BistNode::IRLightCB(const sensor_msgs::IlluminanceConstPtr& ptr)
{
	//ROS_DEBUG("night mode Illuminance=%f",ptr->illuminance);
	//printf ("night mode Illuminance=%f\r\n",ptr->illuminance);
	int illum = static_cast<int>(ptr->illuminance);
    int mPrevCh0 = illum & 0xFFFF;
    printf ("ir light mPrevCh0=%d\r\n",mPrevCh0);

	//ir light off
	if( mIrLightOff && (0 < mPrevCh0) ){
		if( 5 > mIRLightOffCount ){
			mIRLightOffValu += mPrevCh0;
			mIRLightOffCount++;
		}
	}
	//ir light on
	if( mIrLightOn && (0 < mPrevCh0) ){
		if( 5 > mIRLightOnCount ){
			mIRLightOnValu += mPrevCh0;
			mIRLightOnCount++;
		}
	}

	int mPrevOnValu = 0;
	int mPrevOffValu = 0;
	if(0 < mIRLightOnCount){
		mPrevOnValu = mIRLightOnValu/mIRLightOnCount;
	}
	if(0 < mIRLightOffCount){
		mPrevOffValu = mIRLightOffValu/mIRLightOffCount;
	}

	printf("On:%d,Off:%d\r\n",mPrevOnValu,
							mPrevOffValu);
	if( mPrevOnValu > (mPrevOffValu +100) ){
		mIRLight = true;
		std::unique_lock<std::mutex> lck(mtx);
		cv.notify_all();
		printf ("====================\r\n");
	}
}



void BistNode::BatteryStatusCB(const statusConstPtr &s)
{
	mCharging=s->status[2];
	if(0< mCharging ){
		mBattery = true;
		//lock_guard<mutex> lk(mBistMutex);
		std::unique_lock<std::mutex> lck(mtx);
		cv.notify_all();
	}
}

void BistNode::processResetKey(){
	struct input_event ev;
	dzlog_info("processResetKey.");
	if(read(mResetFD, &ev,sizeof(ev)) != sizeof(ev)){
		return;
	}
	if(ev.type == 0){
		return;
	}

	if(ev.value == 1){
		mLastResetTime = ev.time;
	}else{
		int t = plt_timeval_diff_ms(&ev.time, &mLastResetTime);
		if(t >= RESET_KEY_LONG_PRESS_TIME){
		  dzlog_info("handle processResetKey.");
		  mResetKey = true;
		}
	}
}

void BistNode::processPowerKey(){
	  struct input_event ev;
        if(read(mPowerFD,&ev,sizeof(ev))!=sizeof(ev)){
            return;
        }
        if(ev.type==0){
            return;
        }
        if(ev.value==1){
            mLastPowerTime=ev.time;
        }else{
            int t=plt_timeval_diff_ms(&ev.time,&mLastPowerTime);
            if(t>=POWER_KEY_LONGPRESS_TIME){               
				dzlog_info("handle processPowerKey.");
				mPowerKey = true;
            }
        }
}

void BistNode::tofDataCB(const sensor_msgs::RangeConstPtr &r)
{
	static int nTofCnt = 0;
	if (nTofCnt++>5){
		mPCBRange = r->range;
	}
	//printf("r->range:%f\r\n", r->range);
	if(isinf(r->range)){
		printf("tofDataCB isinf!!!!!!!!!\r\n");
		return ;
	}
	if(!isinf(r->range) && ( 0 < r->range ) ){
		mRange = r->range;
		if( ( (BIST_MOVE_DIST-0.2) <mRange) && (mRange<(BIST_MOVE_DIST+0.2)) ){ //50cm
			mBistTof = true;
			std::unique_lock<std::mutex> lck(mtx);
			cv.notify_all();
		}
	}
#if 0
	if( !isinf(r->range) && (0.0 != mDistance) &&
	((mDistance*1.1 > r->range) || (mDistance*0.9 < r->range)) ){
		printf("(%f,%f,%f)\r\n", mDistance*1.1,mDistance*0.9, r->range);
	//if( 0 < r->range){
		mBistTof = true;
		mRange = r->range;
		//lock_guard<mutex> lk(mBistMutex);
		std::unique_lock<std::mutex> lck(mtx);
		cv.notify_all();
	}
#endif
}

void BistNode::MotorWheelCB(const detectConstPtr& obj)
{
	printf("obj->name:%s\r\n",obj->name.c_str());

	if( "home" == obj->name ){
		printf("#####################################\r\n");
		std::unique_lock<std::mutex> lck(mtx);
		cv.notify_all();
		mBistVideoDis = true;
	}
}

void BistNode::odomRelativeCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    Eigen::Vector3d t(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);

    mCurPostion+=mCurPose*t;
    mCurPose=mCurPose*Eigen::Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
	//printf ("odomRelativeCB.\r\n");
}

void BistNode::calDistanceAndAngle(Eigen::Quaterniond& q,Eigen::Vector3d& pos,Eigen::Vector3d &goal,double &angle,double &distance)
{
	Eigen::Vector3d d=q.inverse()*(goal-pos);
	distance=d.norm();
	if(distance<0.1/2){
		angle=0;
		return;
	}
	angle=std::asin(d.y()/distance);
	if(d.x()<0){
		angle=(angle>0?(M_PI-angle):(-M_PI-angle));
	}
	angle-=M_PI_2;
	if(angle<-M_PI){
		angle+=2*M_PI;
	}
}
void BistNode::PCMAnalysis(int type)
{
	mPcmHzs[0] = mPcmHzs[1] = mPcmHzs[2] =0;
	ifstream file(WAVFILE);
	if(!file.is_open()){
		dzlog_error("WAV file[%s] not exist\n",WAVFILE.c_str());
		return;
    }
	int nChan = 2;
    int nFileLen = 0;
    if(1 == type){
		file.seekg(44, ios_base::end);
		nFileLen = file.tellg();
		file.seekg(44,ios_base::beg);
    }else{
		file.seekg(0, ios_base::end);
		nFileLen = file.tellg();
		file.seekg(0,ios_base::beg);
    }
	if (nFileLen < 3 * 44100*nChan+88){
		return;
	}

	int STATE_STEP1 = 0;
	int STATE_STEP2 = 1;
	int mStep = STATE_STEP1;
	//int mCirclePointCount = 0;
	//shared_ptr<char > data = new char[nFileLen];
	//std::shared_ptr<char> data (new <char>(10));
	char data[nFileLen] = {0};
	printf("nFileLen:%d\r\n",nFileLen);
	file.read(data,nFileLen);

    int start = 88;
    int span = 2*44100*nChan;

    for (int j=0; j<3; j++){
        short sh = 0;
        for (int i=start+j*span; i<start+j*span+2*44100 * nChan; i++) {
            short sh1 = data[i];
            sh1 &= 0xff;
            short sh2 = data[++i];
            sh2 <<= 8;

            sh = (short) ((sh1) | (sh2));
            //printf("sh:%x,sh1:%x,sh2:%x\r\n",sh,sh1, sh2);
            if (STATE_STEP1 == mStep) {
                if (sh > 0) {
                    mStep = STATE_STEP2;
                }
            } else if (STATE_STEP2 == mStep) {
                if (sh < 0) {                    
                    mPcmHzs[j]++;
                    mStep = STATE_STEP1;
                    //printf("0mCirclePointCount:%d\r\n",mCirclePointCount);
                }
            }
        }
    }
	
	printf("mPcmHzs:%d, %d, %d\r\n",mPcmHzs[0], mPcmHzs[1], mPcmHzs[2]);
}
void BistNode::PCMgeneration(){
	double x = 0;
	short y = 0;
	int f = 6000;
	int fs = 44100 * 2;//sample rate is 44100 2 channel
	char name[64] = {0};
	int fd = 0;
	
	double w = 2 * M_PI * f;
	double step  = (double)1.0 / (double)fs;
	memset(name, 0, 64);
	sprintf(name, "%s", PCMFILE.c_str());
	fd = open(name, O_CREAT | O_RDWR | O_TRUNC, 0777);
	for(x = 0; x <= 5; x += step) {
	y = sin(w * x) * 16384;
		write(fd, &y, 2);
	}
	close (fd);
}

int BistNode::pcm2wave(const char *pcmpath, int channels, int sample_rate, const char *wavepath)
{
    typedef struct WAVE_HEADER{
        char    fccID[4];      
        unsigned int dwSize;   
        char    fccType[4];     
    }WAVE_HEADER;

    typedef struct WAVE_FMT{
        char    fccID[4];         
        unsigned int  dwSize;     
        short int wFormatTag; 
        short int wChannels;  
        unsigned int  dwSamplesPerSec;
        unsigned int  dwAvgBytesPerSec;/* ==dwSamplesPerSec*wChannels*uiBitsPerSample/8 */
        short int wBlockAlign;//==wChannels*uiBitsPerSample/8
        short int uiBitsPerSample;
    }WAVE_FMT;

    typedef struct WAVE_DATA{
        char    fccID[4];       
        unsigned int dwSize;   //==NumSamples*wChannels*uiBitsPerSample/8
    }WAVE_DATA;

    if(channels==2 || sample_rate==0)
    {
        channels = 2;
        sample_rate = 44100;
    }

    WAVE_HEADER pcmHEADER;
    WAVE_FMT    pcmFMT;
    WAVE_DATA   pcmDATA;

    short int m_pcmData;
    FILE *fp, *fpout;

    fp = fopen(pcmpath, "rb+");
    if(fp==NULL)
    {
        printf("Open pcm file error.\n");
        return -1;
    }
    fpout = fopen(wavepath, "wb+");
    if(fpout==NULL)
    {
        printf("Create wav file error.\n");
        return -1;
    }

    /* WAVE_HEADER */
    memcpy(pcmHEADER.fccID, "RIFF", 4);
    memcpy(pcmHEADER.fccType, "WAVE", 4);
    fseek(fpout, sizeof(WAVE_HEADER), 1);   //1=SEEK_CUR
    /* WAVE_FMT */
    memcpy(pcmFMT.fccID, "fmt ", 4);
    pcmFMT.dwSize = 16;
    pcmFMT.wFormatTag = 0x0001;
    pcmFMT.wChannels = channels;
    pcmFMT.dwSamplesPerSec = 44100;//16000;
    pcmFMT.uiBitsPerSample = 16;
    /* ==dwSamplesPerSec*wChannels*uiBitsPerSample/8 */
    pcmFMT.dwAvgBytesPerSec = pcmFMT.dwSamplesPerSec*pcmFMT.wChannels*pcmFMT.uiBitsPerSample/8;
    /* ==wChannels*uiBitsPerSample/8 */
    pcmFMT.wBlockAlign = pcmFMT.wChannels*pcmFMT.uiBitsPerSample/8;

    fwrite(&pcmFMT, sizeof(WAVE_FMT), 1, fpout);

    /* WAVE_DATA */
    memcpy(pcmDATA.fccID, "data", 4);
    pcmDATA.dwSize = 0;
    fseek(fpout, sizeof(WAVE_DATA), 1);

    fread(&m_pcmData, sizeof(short int), 1, fp);
    while(!feof(fp))
    {
        pcmDATA.dwSize += sizeof(short int);
        fwrite(&m_pcmData, sizeof(short int), 1, fpout);
        fread(&m_pcmData, sizeof(short int), 1, fp);
    }
    pcmHEADER.dwSize = 36 + pcmDATA.dwSize;

    rewind(fpout);
    fwrite(&pcmHEADER, sizeof(WAVE_HEADER), 1, fpout);
    fseek(fpout, sizeof(WAVE_FMT), SEEK_CUR);
    fwrite(&pcmDATA, sizeof(WAVE_DATA), 1, fpout);

    fclose(fp);
    fclose(fpout);

    return 0;
}

void BistNode::BistSaveImage()
{
	mBistVideoJpg = false;
	m_subJPG = mGlobal->subscribe("CoreNode/jpg", 100, &BistNode::JpgCallback, this);
}

void BistNode::BistMatImage()
{
	//mBistVideoJpg = false;
	m_subJPG = mGlobal->subscribe("CoreNode/jpg", 100, &BistNode::JpgMatCb, this);
}

//void BistNode::BistGreyImage()
//{
//	printf ("0 BistGreyImage.\r\n");
//	m_subGrey = mGlobal->subscribe("CoreNode/grey_img", 100, &BistNode::GreyImagCallback, this);
//	printf ("1 BistGreyImage.\r\n");
//}

float BistNode::ImagArticulation(const cv::Mat &image)
{
	#define BLOCK 60
	if (image.empty()) {
		return 0.0;
	}
	cv::Mat imageGrey;
	
	cv::cvtColor(image, imageGrey, CV_RGB2GRAY);
	
	Mat frame = imageGrey;

	int cx = frame.cols/2;
	int cy = frame.rows/2;

	// Go float
	Mat fImage;
	frame.convertTo(fImage, CV_32F);

	// FFT
	cout << "Direct transform...\n";
	Mat fourierTransform;
	dft(fImage, fourierTransform, DFT_SCALE|DFT_COMPLEX_OUTPUT);

	//center low frequencies in the middle
	//by shuffling the quadrants.
	Mat q0(fourierTransform, Rect(0, 0, cx, cy));	   // Top-Left - Create a ROI per quadrant
	Mat q1(fourierTransform, Rect(cx, 0, cx, cy));	   // Top-Right
	Mat q2(fourierTransform, Rect(0, cy, cx, cy));	   // Bottom-Left
	Mat q3(fourierTransform, Rect(cx, cy, cx, cy));	   // Bottom-Right

	Mat tmp; 										   // swap quadrants (Top-Left with Bottom-Right)
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);

	q1.copyTo(tmp);									   // swap quadrant (Top-Right with Bottom-Left)
	q2.copyTo(q1);
	tmp.copyTo(q2);

	// Block the low frequencies
	// #define BLOCK could also be a argument on the command line of course
	fourierTransform(Rect(cx-BLOCK,cy-BLOCK,2*BLOCK,2*BLOCK)).setTo(0);

	//shuffle the quadrants to their original position
	Mat orgFFT;
	fourierTransform.copyTo(orgFFT);
	Mat p0(orgFFT, Rect(0, 0, cx, cy));		 // Top-Left - Create a ROI per quadrant
	Mat p1(orgFFT, Rect(cx, 0, cx, cy)); 	 // Top-Right
	Mat p2(orgFFT, Rect(0, cy, cx, cy)); 	 // Bottom-Left
	Mat p3(orgFFT, Rect(cx, cy, cx, cy));	 // Bottom-Right

	p0.copyTo(tmp);
	p3.copyTo(p0);
	tmp.copyTo(p3);

	p1.copyTo(tmp);									   // swap quadrant (Top-Right with Bottom-Left)
	p2.copyTo(p1);
	tmp.copyTo(p2);

	// IFFT
	cout << "Inverse transform...\n";
	Mat invFFT;
	Mat logFFT;
	double minVal,maxVal;

	dft(orgFFT, invFFT, DFT_INVERSE|DFT_REAL_OUTPUT);

	//img_fft = 20*numpy.log(numpy.abs(img_fft))
	invFFT = cv::abs(invFFT);
	cv::minMaxLoc(invFFT,&minVal,&maxVal,NULL,NULL);

	//check for impossible values
	if(maxVal<=0.0){
		cerr << "No information, complete black image!\n";
		return maxVal;
	}

	cv::log(invFFT,logFFT);
	logFFT *= 20;

	//result = numpy.mean(img_fft)
	cv::Scalar result= cv::mean(logFFT);
	cout << "Result : "<< result.val[0] << endl;

//    Mat finalImage;
//    logFFT.convertTo(finalImage, CV_8U);    // Back to 8-bits
//    imwrite("Result.jpg", finalImage);
//	imwrite("Imput.jpg", imageGrey);
	return result.val[0];
}


void BistNode::JpgCallback(roller_eye::frameConstPtr frame)
{
	//printf("JpgCallback.\r\n");
    FILE *fp_save = nullptr;
    if (frame->data.size() <= 0) {
        PLOG_DEBUG(REC_TAG, "frame data is empty.");
        return;
    }
    fp_save = fopen(BistImage.c_str(), "wb");
    fwrite(&frame->data[0], 1, frame->data.size(), fp_save);
    dzlog_info("Bist video image save:%s.",BistImage.c_str());
    fclose(fp_save);
    m_subJPG.shutdown();

    mBistVideoJpg = true;
	//lock_guard<mutex> lk(mBistMutex);
	std::unique_lock<std::mutex> lck(mtx);
	cv.notify_all();
}

void BistNode::JpgMatCb(roller_eye::frameConstPtr frame)
{
	printf("JpgMatCb.\r\n");
    FILE *fp_save = nullptr;
    if (frame->data.size() <= 0) {
        PLOG_DEBUG(REC_TAG, "frame data is empty.");
        return;
    }

	cv::Mat inputImg = cv::imdecode(frame->data, 1);
	printf("imdecode\r\n");
	if(!inputImg.empty()){
		printf("cvtColor\r\n");
		cvtColor(inputImg,m_BistGrey,CV_BGR2GRAY);
		if(!m_BistGrey.empty()){
			printf("resize\r\n");
			cv::resize(m_BistGrey, m_BistGrey, cv::Size(1280,720), 0, 0, cv::INTER_NEAREST);
			if(m_BistGrey.empty()){
				printf("m_BistGrey.empty\r\n");
				return;
			}
		}
	}
    m_subJPG.shutdown();

	std::unique_lock<std::mutex> lck(mtx);
	cv.notify_all();
}

void   BistNode::BistMemDetect()
{
    FILE* stream = NULL;
    int nMem = 0;
    char *cmd = "cat /proc/meminfo | grep MemTotal | awk '{print $2}'";
    stream = popen(cmd, "r");
    if (stream != NULL){
    	char buf[512]={0};
        while (!feof(stream)){
           if (fread(buf, sizeof(char), sizeof(buf), stream)>0){
        		nMem = atoi(buf);
		   }
        }
    }

    if (stream != NULL){
        pclose(stream);
    }

	if (nMem>MEM_SIZE){
		mMemery = true;
	}
	if(!mMemery){
		dzlog_error("Bist MemDetect status:%s.",mMemery ? "Passed" : "Failure");
		//LEDStatus(BIST_BATTERY_ERROR);
	}else{
		dzlog_info("Bist MemDetect status:%s.",mMemery ? "Passed" : "Failure");
	}

	printf("BistMemDetect: %d %d %d\r\n",nMem, MEM_SIZE,mMemery);
}

void BistNode::BistDiskDetect()
{
    FILE* stream = NULL;
    char *cmd = "fdisk -l | grep -w \"/dev/mmcblk0\" | awk '{print $3}'";
    stream = popen(cmd, "r");
   	float fDisk = 0.0;
    if (stream != NULL){
    	char buf[512]={0};
        while (!feof(stream)){
            if (fread(buf, sizeof(char), sizeof(buf), stream)>0){
            	fDisk = atof(buf);
			}
        }
    }
    if (stream != NULL){
        pclose(stream);
    }
	if (fDisk > DISK_SIZE){
		mDisk = true;
	}
	if(!mDisk){
		dzlog_error("Bist MemDetect status:%s.",mDisk ? "Passed" : "Failure");
		//LEDStatus(BIST_BATTERY_ERROR);
	}else{
		dzlog_info("Bist MemDetect status:%s.",mDisk ? "Passed" : "Failure");
	}	
	printf("BistDiskDetect: %f %f %d\r\n",fDisk, DISK_SIZE,mDisk);
}

void BistNode::BistDriveMotor()
{
	if (mMotorWheel){
		return;
	}
	auto th = std::thread([this]{
		mMotorWheel = true;
		int nCnt = 0;
		while(mMotorWheel){
			geometry_msgs::Twist vel;
			vel.linear.x = 0;		
			if (nCnt>40){
				nCnt = 0;
				vel.linear.y = 0;
			}else if (nCnt++>20){
				vel.linear.y = -0.15;
			}else{
				vel.linear.y = 0.15;
			}		
			mCmdVel3.publish(vel);
			usleep(100*1000);	
		
		}
	});
	th.detach();

}

#if 0
void BistNode::GreyImagCallback(const sensor_msgs::ImageConstPtr& msg/*const sensor_msgs::Image& msg*/)
{
    //FILE *fp_save = nullptr;
    printf("0 GreyImagCallback.");
    m_BistGrey=cv_bridge::toCvShare(msg, "mono8")->image;
    cv::imwrite("BistGrey.jpg",m_BistGrey);
	printf("GreyImagCallback.");
	std::unique_lock<std::mutex> lck(mtx);
	cv.notify_all();
}
#endif


int main(int argc, char **argv)
{
    ros::init(argc, argv, "BistNode");
    ros::console::set_logger_level("BistNode",ros::console::levels::Debug);

    ros::NodeHandle nh;
	
    g_BistNode = shared_ptr<BistNode>(new BistNode());
    ros::spin();
    return 0;
}
