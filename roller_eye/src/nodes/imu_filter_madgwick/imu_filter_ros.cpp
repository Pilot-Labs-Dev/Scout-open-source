/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "roller_eye/nav_cancel.h"
using namespace roller_eye;

#include "imu_filter_madgwick/imu_filter_ros.h"
#include "imu_filter_madgwick/stateless_orientation.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<cmath>
#include<fstream>

///<<< Use gyro.z * dt as increment of Yaw
#define	USE_SIMPLE_YAW_ESTIMATION	1
///>>>
#define USE_IMU_TOPIC_WATCHDOG	 0   ///<<<1

#define TEST_SAVE_FILE	0	///1	///<<< 0 for disable file log    ///<<< define 1 if enabled
#define LOG_ARRAY_SIZE	(65536*2)

using namespace std;
using namespace Eigen;
using namespace geometry_msgs;
using namespace sensor_msgs;

static const Vector3d G(0.0,0.0,G_VALUE);
const char tab = '\t', cr = '\r';
#define LOG_CYCLE_COUNT    10

#if TEST_SAVE_FILE
ofstream imuOrg;
ofstream imuCab;
ofstream imuCab1;
ofstream imuLog;

TwistStamped logArray[LOG_ARRAY_SIZE];
int logIndex = 0, logCount =0;
int fileCount =0;
char logFileName[255];
#endif
ofstream fLog;

ImuFilterRos::ImuFilterRos(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private),
	mCALIBRATE_MAX_CNT(500),
	mCALIBRATE_SKIP_CNT(300),
	mLogCycleCount (mCfg.getImuFilterLogCycleCount()),
	mOffsetCycleCount (mCfg.getImuFilterOffsetCycleCount()),
	mOffsetThreshold ( mCfg.getImuFilterOffsetThreshold()),
	initialized_(false)
{
	fLog.open("/run/log/imuFilter.log");
	if (!fLog.is_open()) {
		cerr << "Create file failed:" << "imuFilter.txt" << std::endl;
		// ROS_INFO("Open log file imuFilter.txt failed!");
		cerr << "mLogCycleCount: " << mLogCycleCount << endl;
		cerr << "mOffsetCycleCount: " << mOffsetCycleCount << endl;
		cerr << "mOffsetThreshold: " << mOffsetThreshold << endl;
	}
	else
	{
        fLog << "[Filter config params:]"<< endl;
		fLog << "mLogCycleCount: " << mLogCycleCount << endl;
		fLog << "mOffsetCycleCount: " << mOffsetCycleCount << endl;
		fLog << "mOffsetThreshold: " << mOffsetThreshold << endl;
		fLog << endl;
	}

    // **** get paramters
    if (!nh_private_.getParam("stateless", stateless_)) stateless_ = false;
    if (!nh_private_.getParam("use_mag", use_mag_)) use_mag_ = true;
    if (!nh_private_.getParam("publish_tf", publish_tf_)) publish_tf_ = true;
    if (!nh_private_.getParam("reverse_tf", reverse_tf_)) reverse_tf_ = false;
    if (!nh_private_.getParam("fixed_frame", fixed_frame_))
        fixed_frame_ = "odom";
    if (!nh_private_.getParam("constant_dt", constant_dt_)) constant_dt_ = 0.0;
    if (!nh_private_.getParam("remove_gravity_vector", remove_gravity_vector_))
        remove_gravity_vector_ = false;
    if (!nh_private_.getParam("publish_debug_topics", publish_debug_topics_))
        publish_debug_topics_ = false;

    ///<<< temp diable use_mag_ for debug
	use_mag_ = false;
	///<<< force publish debug topics (RPY)
    publish_debug_topics_ = true;

    double yaw_offset = 0.0;
    if (!nh_private_.getParam("yaw_offset", yaw_offset)) yaw_offset = 0.0;
    double declination = 0.0;
    if (!nh_private_.getParam("declination", declination)) declination = 0.0;
    // create yaw offset quaternion
    yaw_offset_total_ = yaw_offset - declination;
    yaw_offsets_.setRPY(
        0, 0,
        yaw_offset_total_);  // Create this quaternion for yaw offset (radians)

    std::string world_frame;
    if (!nh_private_.getParam("world_frame", world_frame)) world_frame = "enu";

    if (world_frame == "ned")
    {
        world_frame_ = WorldFrame::NED;
    } else if (world_frame == "nwu")
    {
        world_frame_ = WorldFrame::NWU;
    } else if (world_frame == "enu")
    {
        world_frame_ = WorldFrame::ENU;
    } else
    {
        ROS_ERROR("The parameter world_frame was set to invalid value '%s'.",
                  world_frame.c_str());
        ROS_ERROR("Valid values are 'enu', 'ned' and 'nwu'. Setting to 'enu'.");
        world_frame_ = WorldFrame::ENU;
    }
    filter_.setWorldFrame(world_frame_);

    // check for illegal constant_dt values
    if (constant_dt_ < 0.0)
    {
        ROS_FATAL("constant_dt parameter is %f, must be >= 0.0. Setting to 0.0",
                  constant_dt_);
        constant_dt_ = 0.0;
    }

    // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
    // otherwise, it will be constant
    if (constant_dt_ == 0.0)
        ROS_INFO("Using dt computed from message headers");
    else
        ROS_INFO("Using constant dt of %f sec", constant_dt_);

    if (remove_gravity_vector_)
        ROS_INFO("The gravity vector will be removed from the acceleration");
    else
        ROS_INFO("The gravity vector is kept in the IMU message.");

    // **** register dynamic reconfigure
    config_server_.reset(new FilterConfigServer(nh_private_));
    FilterConfigServer::CallbackType f =
        boost::bind(&ImuFilterRos::reconfigCallback, this, _1, _2);
    config_server_->setCallback(f);

    // **** register publishers
    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(
        ros::names::resolve("imu") + "/data", 5);
#ifdef USE_SIMPLE_YAW_ESTIMATION
    // **** register publishers
    transform_publisher_ = nh_.advertise<geometry_msgs::TransformStamped>(
        ros::names::resolve("imu") + "/transform", 5);
#else
    if (publish_debug_topics_)
    {
        rpy_filtered_debug_publisher_ =
            nh_.advertise<geometry_msgs::Vector3Stamped>(
                ros::names::resolve("imu") + "/rpy/filtered", 5);

        rpy_raw_debug_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
            ros::names::resolve("imu") + "/rpy/raw", 5);
    }
#endif
    // **** register subscribers
    // Synchronize inputs. Topic subscriptions happen on demand in the
    // connection callback.
    int queue_size = 5;

///    imu_subscriber_.reset(new ImuSubscriber(
///        nh_, ros::names::resolve("imu") + "/data_raw", queue_size));
	///<<< change IMU subscirbe topic
    imu_subscriber_.reset(new ImuSubscriber(
        nh_, ros::names::resolve("SensorNode") + "/imu", queue_size));

    if (use_mag_)
    {
        mag_subscriber_.reset(new MagSubscriber(
            nh_, ros::names::resolve("imu") + "/mag", queue_size));

        sync_.reset(new Synchronizer(SyncPolicy(queue_size), *imu_subscriber_,
                                     *mag_subscriber_));
        sync_->registerCallback(
            boost::bind(&ImuFilterRos::imuMagCallback, this, _1, _2));
    } else
    {
        imu_subscriber_->registerCallback(&ImuFilterRos::imuCallback, this);
    }

    check_topics_timer_ = nh_.createTimer(
        ros::Duration(10.0), &ImuFilterRos::checkTopicsTimerCallback, this);

	mImuFilterReset=nh.advertiseService("/imu/filter_reset",&ImuFilterRos::imuFilterReset,this);
	mImuFilterCalibStatus=nh.advertiseService("/imu/filter_calib_status",&ImuFilterRos::imuFilterCalibStatus,this);
	mImuFilterStart = nh.advertiseService("/imu/filter_start",&ImuFilterRos::imuFilterStart,this);
	mImuFilterStop = nh.advertiseService("/imu/filter_stop",&ImuFilterRos::imuFilterStop,this);

#if TEST_SAVE_FILE && !USE_IMU_TOPIC_WATCHDOG
	mFilterRunning = true;    ///<<< default running for file loging
#else
	mFilterRunning = false;    ///<<< Pause for listening to imuFilterStart
#endif
	if( false == mFilterRunning  )
		if(imu_subscriber_)
			imu_subscriber_->unsubscribe();
#if TEST_SAVE_FILE
	imuOrg.open("/run/log/imuOrg.txt");
	if (!imuOrg.is_open()) {
		cerr << "Create file failed:" << "imuOrg.txt" << std::endl;
	}
	imuCab.open("/run/log/imuCab.txt");
	if (!imuCab.is_open()) {
		cerr << "Create file failed:" << "imuCab.txt" << std::endl;
	}
	imuCab1.open("/run/log/imuCab1.txt");
	if (!imuCab1.is_open()) {
		cerr << "Create file failed:" << "imuCab1.txt" << std::endl;
	}
#endif
}

ImuFilterRos::~ImuFilterRos()
{
    // ROS_INFO("Destroying ImuFilter");

    // Explicitly stop callbacks; they could execute after we're destroyed
    check_topics_timer_.stop();
#if TEST_SAVE_FILE
	imuOrg.flush();
	imuCab.flush();
	imuCab1.flush();
	imuOrg.close();
	imuCab.close();
	imuCab1.close();
#endif
	fLog.flush();
	fLog.close();
}

bool ImuFilterRos::imuCalibrate(const ImuMsg::ConstPtr& msg)
{
	static Vector3d v_acc_offset(9.0,9.0,9.0);
	static Vector3d v_gyro_offset(9.0,9.0,9.0);
	Vector3d accel(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    Vector3d gyro(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);

	if (0 == m_calibrate_cnt)   ///<<< Reset acc and gyro offset...
	{
		v_acc_offset  =Vector3d(0.0,0.0,0.0);
		v_gyro_offset =Vector3d(0.0,0.0,0.0);
#if TEST_SAVE_FILE
		clog << "Gyro and accl offset reset:" <<endl;
		clog << "Gyro:"<< endl << v_gyro_offset << endl;
		clog << "Accl:"<< endl << v_acc_offset << endl;
		fLog << "[Gyro and accl offset reset:]" <<endl;
		fLog << "Gyro:"<< endl << v_gyro_offset << endl;
		fLog << "Accl:"<< endl << v_acc_offset << endl << endl;
#endif
	}
	if(m_calibrate_cnt++<mCALIBRATE_MAX_CNT){
		// cout << m_calibrate_cnt ;  /// << "\t";
        if (m_calibrate_cnt > mCALIBRATE_SKIP_CNT)
		{
			v_acc_offset  += (accel-G);
			v_gyro_offset += gyro;
		}
	#if TEST_SAVE_FILE
		else
			cout <<  "(skip)";
		cout << tab;
	#endif

        if(m_calibrate_cnt==mCALIBRATE_MAX_CNT){
            v_acc_offset /= (mCALIBRATE_MAX_CNT-mCALIBRATE_SKIP_CNT);
            v_gyro_offset /= (mCALIBRATE_MAX_CNT-mCALIBRATE_SKIP_CNT);

			m_acc_offset.x = v_acc_offset.x();
			m_acc_offset.y = v_acc_offset.y();
			m_acc_offset.z = v_acc_offset.z();
			m_gyro_offset.x = v_gyro_offset.x();
			m_gyro_offset.y = v_gyro_offset.y();
			m_gyro_offset.z = v_gyro_offset.z();

	#if TEST_SAVE_FILE
            std::cout<< endl <<"gyro offset:\n"<<m_gyro_offset<<std::endl;
            std::cout<<"acc offset:\n"<<m_acc_offset<<std::endl;
	#endif

			mImuCalibrated = true;
	#if TEST_SAVE_FILE
			fLog << "[New Gyro and accl offset:]" <<endl;
			fLog << "Gyro offset:"<< endl << v_gyro_offset << endl;
			fLog << "Accl offset:"<< endl << v_acc_offset << endl << endl;
	#endif
        }
        return true;
    }
    return false;
}

void ImuFilterRos::imuCallback(const ImuMsg::ConstPtr& imu_msg_raw)
{
#if USE_IMU_TOPIC_WATCHDOG
	 if(check_topics_timer_)
	 	check_topics_timer_.stop();
#endif
	if (false == mFilterRunning)
		return;

#if USE_IMU_TOPIC_WATCHDOG
	 if(check_topics_timer_)
	 {
		 mWdgCnt = 0 ;
		 check_topics_timer_.start();
	 }
#endif

	int N_short =2 , N_long = 128;
	double Vdc_residue_limit = 0.002;
	static double avgDC_short = 0.0;
	static double avgDC_long = 0.0;
	static double prev_data = 0.0;
	double delta_GyroZ = 0.0;

	boost::mutex::scoped_lock lock(mutex_);

#if TEST_SAVE_FILE
	 logArray[logIndex].header.seq = m_calibrate_cnt;
	 logArray[logIndex].header.stamp = imu_msg_raw->header.stamp;
	 logArray[logIndex].twist.linear.x = imu_msg_raw->angular_velocity.z;
#else
	/// cout << imu_msg_raw->header.stamp << tab;
#endif

    if(imuCalibrate(imu_msg_raw)){
	///	gyro_z_dc1 = 0.0;
		prev_data = 0.0;
		avgDC_short =0.0;
		avgDC_long = 0.0;
		/// last_time_  = imu_msg_raw->header.stamp;
#if TEST_SAVE_FILE
		logCount ++ ;   /// logIndex ++;
		logIndex = logCount % LOG_ARRAY_SIZE;
		memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));   /// clear buffer
#endif
		return;
    }

    geometry_msgs::Vector3 ang_vel = imu_msg_raw->angular_velocity;
    geometry_msgs::Vector3 lin_acc = imu_msg_raw->linear_acceleration;

#if TEST_SAVE_FILE
///	logIndex %= LOG_ARRAY_SIZE;
	logArray[logIndex].header.seq = m_calibrate_cnt;
	logArray[logIndex].header.stamp = imu_msg_raw->header.stamp;
	logArray[logIndex].twist.linear.x = ang_vel.z;
#endif

	ang_vel.x -= m_gyro_offset.x;
	ang_vel.y -= m_gyro_offset.y;
	ang_vel.z -= m_gyro_offset.z;
    lin_acc.x -= m_acc_offset.x;
    lin_acc.y -= m_acc_offset.y;
    lin_acc.z -= m_acc_offset.z;

#if TEST_SAVE_FILE
	 logArray[logIndex].twist.linear.y = ang_vel.z;
#endif

	double data = ang_vel.z;
	avgDC_short = avgDC_short - avgDC_short/N_short + data/N_short;
	delta_GyroZ = avgDC_short - ( data - prev_data );
	prev_data = data;

	if (delta_GyroZ > Vdc_residue_limit)
		delta_GyroZ = Vdc_residue_limit;
	///else
	if (delta_GyroZ < -Vdc_residue_limit)
		delta_GyroZ = -Vdc_residue_limit;

	avgDC_long = avgDC_long - avgDC_long/N_long + delta_GyroZ/N_long;
	ang_vel.z = data - avgDC_long;

	///<<< Enable to filter noise, force move on 2D plane...
	ang_vel.x = 0.0;
	ang_vel.y = 0.0;

	lin_acc.x = 0.0;
	lin_acc.y = 0.0;
	lin_acc.z = G.z();
	///>>>  RogerL_2022_09_23

#if TEST_SAVE_FILE
	 logArray[logIndex].twist.linear.z = ang_vel.z;
#endif

    ros::Time time = imu_msg_raw->header.stamp;
    imu_frame_ = imu_msg_raw->header.frame_id;

    if (!initialized_ || stateless_)
    {
        geometry_msgs::Quaternion init_q;
        if (!StatelessOrientation::computeOrientation(world_frame_, lin_acc,
                                                      init_q))
        {
            ROS_WARN_THROTTLE(5.0,
                              "The IMU seems to be in free fall, cannot "
                              "determine gravity direction!");
            return;
        }
        filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

#if TEST_SAVE_FILE
		clog << "[Filter init orientation:] " <<endl << init_q << endl;
		fLog << "[Filter init orientation:] " <<endl << init_q << endl;
#endif
    }

    if (!initialized_)
    {
#if USE_IMU_TOPIC_WATCHDOG
		check_topics_timer_.setPeriod(ros::Duration(2.0)); ///<<< watchdog on IMU datas
#else
        check_topics_timer_.stop();
#endif
        // initialize time
        last_time_ = time;
        initialized_ = true;
    }

    // determine dt: either constant, or from IMU timestamp
    float dt;
    if (constant_dt_ > 0.0)
        dt = constant_dt_;
    else
    {
        dt = (time - last_time_).toSec();
#if TEST_SAVE_FILE
        if (time.isZero())
            ROS_WARN_STREAM_THROTTLE(
                5.0,
                "The IMU message time stamp is zero, and the parameter "
                "constant_dt is not set!"
                    << " The filter will not update the orientation.");
#endif
    }

    last_time_ = time;

#ifdef  USE_SIMPLE_YAW_ESTIMATION
 if(initialized_)
 {
	double currYaw = m_Yaw_Save;	///m_Yaw_Pub ;
	m_Yaw_Save += ang_vel.z * dt;

///	double currYaw = m_Yaw_Save;	///m_Yaw_Pub ;
///
///	mYawBuff[mBufIndex] = currYaw;
///	mBufIndex ++;
///	mBufIndex %=  mYawAverCount; ///20;
///
///	m_Yaw_Pub = avg_angle_nomod(mYawBuff,mYawAverCount);   /// 20
///

	m_Yaw_Pub = m_Yaw_Save;

	geometry_msgs::TransformStamped transform;
	{
	 	 transform.header.stamp = imu_msg_raw->header.stamp;
	 	 transform.header.seq = m_calibrate_cnt;

		 transform.header.frame_id = fixed_frame_;
		 transform.child_frame_id = imu_frame_;

		 transform.transform.translation.x = 0.0;
		 transform.transform.translation.y = 0.0;
		 transform.transform.translation.z = m_Yaw_Pub;

		 transform.transform.rotation.w = cos(m_Yaw_Pub/2.0);
		 transform.transform.rotation.x = 0.0;
		 transform.transform.rotation.y = 0.0;
		 transform.transform.rotation.z = 1.0*sin(m_Yaw_Pub/2.0);
	}
 	transform_publisher_.publish(transform);

#if TEST_SAVE_FILE
	 if(transform.header.stamp != logArray[logIndex].header.stamp )
	 {
		  logCount ++;	///logIndex ++;
		  logIndex =  logCount % LOG_ARRAY_SIZE;
		 memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));	 /// clear buffer
	 }
#endif
#if TEST_SAVE_FILE
	  logArray[logIndex].header.seq = m_calibrate_cnt;
	  logArray[logIndex].header.stamp = transform.header.stamp;
	  logArray[logIndex].twist.angular.z = m_Yaw_Pub;
	  logArray[logIndex].twist.angular.x = currYaw;
	  logArray[logIndex].twist.angular.y = m_Yaw_Pub - currYaw;

	  logCount ++ ; /// logIndex++;
	  logIndex = logCount % LOG_ARRAY_SIZE;
	  memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));  /// clear buffer
#endif

 }
#else
	if (!stateless_)
        filter_.madgwickAHRSupdateIMU(ang_vel.x, ang_vel.y, ang_vel.z,
                                      lin_acc.x, lin_acc.y, lin_acc.z, dt);

    publishFilteredMsg(imu_msg_raw);
    if (publish_tf_) publishTransform(imu_msg_raw);
#endif
}

void ImuFilterRos::imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                                  const MagMsg::ConstPtr& mag_msg)
{
    boost::mutex::scoped_lock lock(mutex_);

    const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
    const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
    const geometry_msgs::Vector3& mag_fld = mag_msg->magnetic_field;

    ros::Time time = imu_msg_raw->header.stamp;
    imu_frame_ = imu_msg_raw->header.frame_id;

	ROS_DEBUG("%s()\n",__func__);

    /*** Compensate for hard iron ***/
    geometry_msgs::Vector3 mag_compensated;
    mag_compensated.x = mag_fld.x - mag_bias_.x;
    mag_compensated.y = mag_fld.y - mag_bias_.y;
    mag_compensated.z = mag_fld.z - mag_bias_.z;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    if (!initialized_ || stateless_)
    {
        // wait for mag message without NaN / inf
        if (!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) ||
            !std::isfinite(mag_fld.z))
        {
            return;
        }

        geometry_msgs::Quaternion init_q;
        if (!StatelessOrientation::computeOrientation(world_frame_, lin_acc,
                                                      mag_compensated, init_q))
        {
            ROS_WARN_THROTTLE(
                5.0,
                "The IMU seems to be in free fall or close to magnetic north "
                "pole, cannot determine gravity direction!");
            return;
        }
        filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
    }

    if (!initialized_)
    {
        check_topics_timer_.stop();

        // initialize time
        last_time_ = time;
        initialized_ = true;
    }

    // determine dt: either constant, or from IMU timestamp
    float dt;
    if (constant_dt_ > 0.0)
        dt = constant_dt_;
    else
    {
        dt = (time - last_time_).toSec();
        if (time.isZero())
            ROS_WARN_STREAM_THROTTLE(
                5.0,
                "The IMU message time stamp is zero, and the parameter "
                "constant_dt is not set!"
                    << " The filter will not update the orientation.");
    }

    last_time_ = time;

    if (!stateless_)
        filter_.madgwickAHRSupdate(ang_vel.x, ang_vel.y, ang_vel.z, lin_acc.x,
                                   lin_acc.y, lin_acc.z, mag_compensated.x,
                                   mag_compensated.y, mag_compensated.z, dt);

    publishFilteredMsg(imu_msg_raw);
    if (publish_tf_) publishTransform(imu_msg_raw);

    if (publish_debug_topics_)
    {
        geometry_msgs::Quaternion orientation;
        if (StatelessOrientation::computeOrientation(
                world_frame_, lin_acc, mag_compensated, orientation))
        {
            tf2::Matrix3x3(tf2::Quaternion(orientation.x, orientation.y,
                                           orientation.z, orientation.w))
                .getRPY(roll, pitch, yaw, 0);
            publishRawMsg(time, roll, pitch, yaw);
        }
    }
}

void ImuFilterRos::publishTransform(const ImuMsg::ConstPtr& imu_msg_raw)
{
    double q0, q1, q2, q3;

	ROS_DEBUG("%s()\n",__func__);

	filter_.getOrientation(q0, q1, q2, q3);
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = imu_msg_raw->header.stamp;
    if (reverse_tf_)
    {
        transform.header.frame_id = imu_frame_;
        transform.child_frame_id = fixed_frame_;
        transform.transform.rotation.w = q0;
        transform.transform.rotation.x = -q1;
        transform.transform.rotation.y = -q2;
        transform.transform.rotation.z = -q3;
    } else
    {
        transform.header.frame_id = fixed_frame_;
        transform.child_frame_id = imu_frame_;
        transform.transform.rotation.w = q0;
        transform.transform.rotation.x = q1;
        transform.transform.rotation.y = q2;
        transform.transform.rotation.z = q3;
    }
    tf_broadcaster_.sendTransform(transform);
}

/**
 * @brief Applies yaw offset quaternion (yaw offset - declination) to the
 *orientation quaternion. Alters the quaternion if there is a yaw offset.
 * @param q0 quaternion x component
 * @param q1 quaternion y component
 * @param q2 quaternion z component
 * @param q3 quaternion w component
 **/
void ImuFilterRos::applyYawOffset(double& q0, double& q1, double& q2,
                                  double& q3)
{

	ROS_DEBUG("%s()\n",__func__);
    if (yaw_offset_total_ != 0.0)
    {
        tf2::Quaternion offset_orientation =
            yaw_offsets_ * tf2::Quaternion(q1, q2, q3, q0);
        offset_orientation.normalize();
        q1 = offset_orientation.x();
        q2 = offset_orientation.y();
        q3 = offset_orientation.z();
        q0 = offset_orientation.w();
		cout << "(q0,q1,q2,q3) = " << q0 << q1 << q2 << q3 << endl ;
    }
}

void ImuFilterRos::publishFilteredMsg(const ImuMsg::ConstPtr& imu_msg_raw)
{
    double q0, q1, q2, q3;

	ROS_DEBUG("%s()\n",__func__);

    filter_.getOrientation(q0, q1, q2, q3);
    // apply yaw offsets
    applyYawOffset(q0, q1, q2, q3);

    // create and publish filtered IMU message
    boost::shared_ptr<ImuMsg> imu_msg =
        boost::make_shared<ImuMsg>(*imu_msg_raw);

    imu_msg->orientation.w = q0;
    imu_msg->orientation.x = q1;
    imu_msg->orientation.y = q2;
    imu_msg->orientation.z = q3;

    imu_msg->orientation_covariance[0] = orientation_variance_;
    imu_msg->orientation_covariance[1] = 0.0;
    imu_msg->orientation_covariance[2] = 0.0;
    imu_msg->orientation_covariance[3] = 0.0;
    imu_msg->orientation_covariance[4] = orientation_variance_;
    imu_msg->orientation_covariance[5] = 0.0;
    imu_msg->orientation_covariance[6] = 0.0;
    imu_msg->orientation_covariance[7] = 0.0;
    imu_msg->orientation_covariance[8] = orientation_variance_;

    if (remove_gravity_vector_)
    {
        float gx, gy, gz;
        filter_.getGravity(gx, gy, gz);
        imu_msg->linear_acceleration.x -= gx;
        imu_msg->linear_acceleration.y -= gy;
        imu_msg->linear_acceleration.z -= gz;
    }

   /// imu_publisher_.publish(imu_msg);
	mOrientation = imu_msg->orientation;

    if (publish_debug_topics_)
    {
        geometry_msgs::Vector3Stamped rpy;
        tf2::Matrix3x3(tf2::Quaternion(q1, q2, q3, q0))
            .getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

        rpy.header = imu_msg_raw->header;
        rpy_filtered_debug_publisher_.publish(rpy);

		/// tf2::Quaternion(q1, q2, q3, q0)
		imu_msg->orientation.w = cos(rpy.vector.z/2.0);
		imu_msg->orientation.x = 0.0;
		imu_msg->orientation.y = 0.0;
		imu_msg->orientation.z = 1.0*sin(rpy.vector.z/2.0);
		imu_publisher_.publish(imu_msg);

#if TEST_SAVE_FILE
	if(imu_msg_raw->header.stamp != logArray[logIndex].header.stamp )
	{
		 logCount ++;  ///logIndex ++;
	 	 logIndex =  logCount % LOG_ARRAY_SIZE;
 	 	memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));   /// clear buffer
	}
#endif
#if TEST_SAVE_FILE
	 logArray[logIndex].header.seq = m_calibrate_cnt;
	 logArray[logIndex].header.stamp = imu_msg_raw->header.stamp;
	 ///logArray[logIndex].twist.angular = rpy.vector;
	 logArray[logIndex].twist.angular.z = m_Yaw_Pub;
	 logArray[logIndex].twist.angular.x = currYaw;
	 logArray[logIndex].twist.angular.y = m_Yaw_Pub - currYaw;

	 logCount ++ ; /// logIndex++;
	 logIndex = logCount % LOG_ARRAY_SIZE;
	 memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));	/// clear buffer
#endif
    }
}

void ImuFilterRos::publishRawMsg(const ros::Time& t, float roll, float pitch,
                                 float yaw)
{
    geometry_msgs::Vector3Stamped rpy;
    rpy.vector.x = roll;
    rpy.vector.y = pitch;
    rpy.vector.z = yaw;
    rpy.header.stamp = t;
    rpy.header.frame_id = imu_frame_;

	ROS_DEBUG("%s()\n",__func__);
    rpy_raw_debug_publisher_.publish(rpy);
}

void ImuFilterRos::reconfigCallback(FilterConfig& config, uint32_t level)
{
    double gain, zeta;
    boost::mutex::scoped_lock lock(mutex_);
    gain = config.gain;
    zeta = config.zeta;

	cout << "&Config:" <<"%p" << (const void *)( & config )<< "level:" << level << endl;
    filter_.setAlgorithmGain(gain);
    filter_.setDriftBiasGain(zeta);
    mag_bias_.x = config.mag_bias_x;
    mag_bias_.y = config.mag_bias_y;
    mag_bias_.z = config.mag_bias_z;
    orientation_variance_ =
        config.orientation_stddev * config.orientation_stddev;
}

void ImuFilterRos::checkTopicsTimerCallback(const ros::TimerEvent& tEvt)
{
#if  USE_IMU_TOPIC_WATCHDOG
	 /// static int t_cnt = 0;
      if (0== (++ mWdgCnt % 2) && initialized_)
      	{
			check_topics_timer_.stop();
			fLog << "IMU Topic Watchdog activated:(Restart SensorNode) !! @ " << ros::Time::now() << endl << flush ;
	  		system("sudo killall sensors_node");    ///<<< try to restart SensorNode
			sleep(3);    ///<<< wait for process restart
			if(imu_subscriber_)
				imu_subscriber_->subscribe();
			mWdgCnt = 0;
			/// initialized_ = false;
			check_topics_timer_.setPeriod(ros::Duration(10.0));
			check_topics_timer_.start();
			return;
      	}
#endif
    if (use_mag_)
        ROS_WARN_STREAM("Still waiting for data on topics "
                        << ros::names::resolve("imu") << "/data_raw"
                        << " and " << ros::names::resolve("imu") << "/mag"
                        << "...");
    else
        ROS_WARN_STREAM("Still waiting for data on topic "
                        << ros::names::resolve("imu") << "/data_raw"
                        << "...");
}

bool ImuFilterRos::imuFilterCalibStatus(getimu_patrolcalib_statusRequest& req,getimu_patrolcalib_statusResponse& res)
{
	res.ret = (m_calibrate_cnt < mCALIBRATE_MAX_CNT);
	return true;
}
#if TEST_SAVE_FILE
bool logOneRecord(int recNo,TwistStamped record, TwistStamped prevRec )
{
	if (0 == record.header.seq)  ///Imu Filter Start
	{
		if(imuLog.is_open())
		{
			imuLog.flush();
			imuLog.close();
		}
		fileCount ++;
		sprintf(logFileName,"/run/log/imuLog_%03d.txt",fileCount);
		imuLog.open(logFileName);
		if(!imuLog.is_open())
			return false;
	}
	imuLog << recNo << tab << record.header.seq <<  tab << fixed <<record.header.stamp << tab << (record.header.stamp -prevRec.header.stamp)
		<< tab << setw(13) << record.twist.linear.x << tab << record.twist.linear.y << tab <<record.twist.linear.z
		<< tab	<< record.twist.angular.x << tab << record.twist.angular.y << tab <<record.twist.angular.z << tab << tf2Degrees(record.twist.angular.z)
		<< cr << endl;

	if(!imuLog.is_open())
		return false;
	return true;
}
#endif
bool ImuFilterRos::imuFilterReset(nav_cancelRequest& req,nav_cancelResponse& res)
{
	clog << "Imu filter reset Orientation to ( 1.0 , 0.0, 0.0, 0.0) @" << __DATE__ "#" __TIME__ << endl;
	filter_.setOrientation(1.0,0.0,0.0,0.0);
	mImuCalibrated = false;
    m_calibrate_cnt=0;
	/// m_gyro_offset = Vector3d(0.0, 0.0, 0.0);
	/// m_acc_offset = Vector3d(0.0, 0.0, 0.0);
	m_acc_offset.x = 0.0;
	m_acc_offset.y = 0.0;
	m_acc_offset.z = 0.0;
	m_gyro_offset.x = 0.0;
	m_gyro_offset.y = 0.0;
	m_gyro_offset.z = 0.0;
	m_Yaw_Pub = 0.0;
	m_Yaw_Save = 0.0;
#if TEST_SAVE_FILE
/// Save only one log...
	logCount = 0;
	logIndex = 0;
	memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));	/// clear buffer
#endif
	return true;
}
bool ImuFilterRos::imuFilterStart(nav_cancelRequest& req,nav_cancelResponse& res)
{
	mFilterRunning = true;
	if(imu_subscriber_)
		imu_subscriber_->subscribe();
	return true;
}
bool ImuFilterRos::imuFilterStop(nav_cancelRequest& req,nav_cancelResponse& res)
{
	mFilterRunning = false;
	if(imu_subscriber_)
		imu_subscriber_->unsubscribe();
#if TEST_SAVE_FILE
		if(logCount > 0 )
		{
			if (!imuCab.is_open()) {
				imuCab.open("/run/imuOrg.txt");
			if (!imuCab.is_open()) {
				cerr << "Create file failed:" << "imuCab.txt" << std::endl;
				return false;
				}
			}

			int i =0;
			///if(m_calibrate_cnt > LOG_ARRAY_SIZE)
			if(logCount > LOG_ARRAY_SIZE)  ///wrap around
				for (i=logIndex;i<LOG_ARRAY_SIZE;i++)
			{
	           logOneRecord(i, logArray[i], logArray[i-1]);
			}
			imuCab.flush();

			for (i=0;i<logIndex;i++)
			{
				if ( 0 == i)
				{
					if (logCount > LOG_ARRAY_SIZE)
						logOneRecord(i, logArray[i], logArray[LOG_ARRAY_SIZE-1]);
					else
						logOneRecord(i, logArray[i], logArray[i]);
				}
				else
					logOneRecord(i, logArray[i], logArray[i-1]);

			}
			/// imuCab.flush();
			imuLog.flush();
			imuLog.close();
			logIndex = 0;   /// Reset save pointer
			memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));   /// clear buffer
			return true;
		}

#endif
	return true;
}

