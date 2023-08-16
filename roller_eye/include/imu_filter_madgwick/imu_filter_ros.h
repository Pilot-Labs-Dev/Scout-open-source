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

#ifndef IMU_FILTER_MADWICK_IMU_FILTER_ROS_H
#define IMU_FILTER_MADWICK_IMU_FILTER_ROS_H


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "tf2_ros/transform_broadcaster.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <tf2/LinearMath/Quaternion.h>

#include "roller_eye/nav_cancel.h"
#include "roller_eye/getimu_patrolcalib_status.h"
#include "roller_eye/plt_config.h"

#include "imu_filter_madgwick/imu_filter.h"
#include "imu_filter_madgwick/ImuFilterMadgwickConfig.h"

#define G_VALUE     9.7833

using namespace sensor_msgs;

class ImuFilterRos
{
    typedef sensor_msgs::Imu ImuMsg;
    typedef sensor_msgs::MagneticField MagMsg;

    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>
        SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;

    typedef Imu_filter_madgwick::ImuFilterMadgwickConfig FilterConfig;
    typedef dynamic_reconfigure::Server<FilterConfig> FilterConfigServer;

  public:
    ImuFilterRos(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~ImuFilterRos();
  	bool imuFilterReset(nav_cancelRequest& req,nav_cancelResponse& res);

  private:
    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    double yaw_offset_total_;

    boost::shared_ptr<ImuSubscriber> imu_subscriber_;
    boost::shared_ptr<MagSubscriber> mag_subscriber_;
    boost::shared_ptr<Synchronizer> sync_;
    tf2::Quaternion yaw_offsets_;

    ros::Publisher rpy_filtered_debug_publisher_;
    ros::Publisher rpy_raw_debug_publisher_;
    ros::Publisher imu_publisher_;
	ros::Publisher transform_publisher_;
	
	ros::ServiceServer mImuFilterReset;
	ros::ServiceServer mImuFilterCalibStatus;
	ros::ServiceServer mImuFilterStart;
	ros::ServiceServer mImuFilterStop;
	
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    boost::shared_ptr<FilterConfigServer> config_server_;
    ros::Timer check_topics_timer_;

    // **** paramaters
    WorldFrame::WorldFrame world_frame_;
    bool use_mag_;
    bool stateless_;
    bool publish_tf_;
    bool reverse_tf_;
    std::string fixed_frame_;
    std::string imu_frame_;
    double constant_dt_;
    bool publish_debug_topics_;
    bool remove_gravity_vector_;
    geometry_msgs::Vector3 mag_bias_;
    double orientation_variance_;

	///<<< Calibration params
	int m_calibrate_cnt=0;
	int mCALIBRATE_MAX_CNT=200;
	int mCALIBRATE_SKIP_CNT=100;
	geometry_msgs::Vector3 m_acc_offset ;
	geometry_msgs::Vector3 m_gyro_offset;
	bool mImuCalibrated=false;
	///>>>>

	// **** state variables
    boost::mutex mutex_;
    bool initialized_;
    ros::Time last_time_;
	double m_Yaw_Pub = 0.0;
	double m_Yaw_Save = 0.0;
	Imu::_orientation_type mOrientation;

    // **** filter implementation
    ImuFilter filter_;
	bool mFilterRunning = true;

	DeviceDefaultConfig mCfg;
	int mLogCycleCount; 		///>>>= mcfg.getImuFilterLogCycleCount();
	int mOffsetCycleCount; 	 	///>>>=mcfg.getImuFilterOffsetCycleCount();
	double mOffsetThreshold; 	///>>>= mcfg.getImuFilterOffsetThreshold();
	int mWdgCnt = 0;    ///for watchdog timer

    // **** member functions
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mav_msg);

    void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);

    void publishFilteredMsg(const ImuMsg::ConstPtr& imu_msg_raw);
    void publishTransform(const ImuMsg::ConstPtr& imu_msg_raw);

    void publishRawMsg(const ros::Time& t, float roll, float pitch, float yaw);

    void reconfigCallback(FilterConfig& config, uint32_t level);
    void checkTopicsTimerCallback(const ros::TimerEvent&);

    void applyYawOffset(double& q0, double& q1, double& q2, double& q3);
	bool imuCalibrate(const ImuMsg::ConstPtr& msg);
	bool imuFilterCalibStatus(getimu_patrolcalib_statusRequest& req,getimu_patrolcalib_statusResponse& res);
	bool imuFilterStart(nav_cancelRequest& req,nav_cancelResponse& res);
	bool imuFilterStop(nav_cancelRequest& req,nav_cancelResponse& res);

};

#endif  // IMU_FILTER_IMU_MADWICK_FILTER_ROS_H
