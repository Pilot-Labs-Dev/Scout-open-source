#include<iostream>
#include<cstdlib>
#include<cstdio>
#include<fstream>
#include<memory>
#include "ros/ros.h"
#include"sensor_msgs/Imu.h"
#include"roller_eye/motor.h"
#include"roller_eye/plt_assert.h"

#define CALC_BIAS_COUNT             200
#define CALC_SPEED_COUNT         1000
#define CALC_SPEED_SKIP_COUNT       200
#define CALC_SLEEP_COUNTER       200
#define CALC_HZ_DELTA                0.1

using namespace roller_eye;

static shared_ptr<Motor> g_motor;

static void setMotorSpeed(float hz)
{
     switch (g_motor->GetDriverType()){
            case MotorDriverType::FS8003:
                plt_assert(g_motor->setParam(0,hz,MOTOR_STATUS_POSITIVE)==0);
                plt_assert(g_motor->setParam(1,hz,MOTOR_STATUS_POSITIVE)==0);
                plt_assert(g_motor->setParam(2,hz,MOTOR_STATUS_NEGTIVE)==0);
                plt_assert(g_motor->setParam(3,hz,MOTOR_STATUS_NEGTIVE)==0);
            break;
            default:
                plt_assert(g_motor->setParam(0,hz,MOTOR_STATUS_POSITIVE)==0);
                plt_assert(g_motor->setParam(1,hz,MOTOR_STATUS_NEGTIVE)==0);
                plt_assert(g_motor->setParam(2,hz,MOTOR_STATUS_POSITIVE)==0);
                plt_assert(g_motor->setParam(3,hz,MOTOR_STATUS_NEGTIVE)==0);
            break;
        }
    plt_assert(g_motor->flush()==0);
}
static void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    static double bias=0.0;
    static bool biasOk=false;
    static int count=0;
    static float speed=0.1;
    static double w=0.0;
    static bool active=false;

    if(!active){
        if(++count==CALC_SLEEP_COUNTER){
            active=true;
            count=0;
        }
        return;
    }

    if(!biasOk){
        bias+=msg->angular_velocity.z;
        if(++count==CALC_BIAS_COUNT){
            bias/=CALC_BIAS_COUNT;
            std::cout<<"z bias:"<<bias<<std::endl;
            biasOk=true;
            count=0;
        }
        return;
    }

    if(count++==0){
        w=0.0;
        setMotorSpeed(speed);
    }
    if(count<CALC_SPEED_SKIP_COUNT){
        return;
    }
    w+=(msg->angular_velocity.z-bias);
    if(count==CALC_SPEED_COUNT){
        w/=(CALC_SPEED_COUNT-CALC_SPEED_SKIP_COUNT+1);
        printf("%f->%f\n",speed,w*(MACC_CAR_WIDTH/2+MACC_CAR_HEIGHT/2)*(1.0/(MACC_PI*MACC_ROLL_DIAMETER)));
        setMotorSpeed(0);
        speed+=CALC_HZ_DELTA;
        if(speed > MACC_MOTOR_MAX_F+0.5){
            ros::shutdown();
            return;
        }
        active=false;
        count=0;
    }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "MotorTuneNode");
  g_motor=shared_ptr<Motor>(createMotor());
  plt_assert(g_motor!=nullptr);
  plt_assert(g_motor->powerON()==0);
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("SensorNode/imu", 1000, imuCallback);
  ros::spin();
  plt_assert(g_motor->powerOFF()==0);
  ROS_INFO("motor tune done!!!");
  return 0;
}