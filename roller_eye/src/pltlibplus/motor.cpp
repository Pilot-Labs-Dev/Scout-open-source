#include<string>
#include<fstream>
#include<fcntl.h>
#include<cmath>
#include"plt_assert.h"
#include"motor.h"
#include"plt_tools.h"
#include"plt_config.h"

#define MOTOR_TAG   "motor"
#define MOTOR_SPEED_EQUAL(v1,v2)     (std::abs((v1)-(v2))<5.0e-2)
#define MOTOR_SPEED_ZERO(v)        (std::abs(v)<1.0e-3)
#define MATCHING_FACTOR             -1

namespace roller_eye{
    static const string PROC_PATH="/proc/driver/";

    Motor::Motor()
    {
        reset();
    }
    Motor::~Motor()
    {
    }

    int Motor::setParam(int idx,float speed,MotorStatus status)
    {
        if(idx<0||idx>=MOTOR_NUM){
            pset_errno(PEPAR);
            return -1;
        }

        //std::cout<<"setParam: "<<idx<<","<<speed<<","<<status<<endl;
        mSpeedChanged[idx]=(!MOTOR_SPEED_ZERO(speed) && MOTOR_SPEED_EQUAL(mParam[idx].speed,speed))?0:1;
        if(mSpeedChanged[idx]){
            mParam[idx].speed=speed;
        }
        mDirChanged[idx]=(mParam[idx].status==status||MOTOR_SPEED_ZERO(speed))?0:1;//if speed == 0 ,do not change dir
        if(mDirChanged[idx]){
            mParam[idx].status=status;
        }
        return 0;
    }

    int Motor::powerON()
    {
        reset();
        return powerOn();
    }
    int Motor::powerOFF()
    {
        reset();
        return powerOff();
    }
    void Motor::reset()
    {
        for(int i=0;i<MOTOR_NUM;i++){
            mParam[i].speed=-10.0;
            mParam[i].status=MOTOR_STATUS_MAX;
        }
        memset(&mSpeedChanged,0,sizeof(mSpeedChanged));
        memset(&mDirChanged,0,sizeof(mDirChanged));
    }

    MaccumMotor::MaccumMotor()
    {
        DeviceDefaultConfig cfg;
        MOROR_SET_DIR_DELAY=cfg.getMotorSetDirDelay();

        //PLOG_INFO(MOTOR_TAG,"Motor Change dir delay=%d\n",MOROR_SET_DIR_DELAY);
    }
    MaccumMotor::~MaccumMotor()
    {

    }

    int MaccumMotor::powerOn()
    {
        ofstream ctl(PROC_PATH+"motor");
        plt_assert(ctl.is_open());
        ctl<<"1"<<std::flush;
        return ctl.fail()?-1:0;
    }
    int MaccumMotor::powerOff()
    {
        ofstream ctl(PROC_PATH+"motor");
        plt_assert(ctl.is_open());
        ctl<<"0"<<std::flush;
        return ctl.fail()?-1:0;
    }
    /*
    High Level motor layout is
                         front
                1               0
left                                    right
                3               2
                     back
    Driver Level layout is
                             front
                1               0(r)
left                                    right
                2               3(r)
                     back
index map is:[0,2,1,3]
dir map [1,0,0,1]
    */
    int MaccumMotor::setDir()
    {
        static int layoutMap[]={0,1,3,2};
        static int dirMap[]={1,0,0,1};
        int dir,i,j;

        ofstream ctl(PROC_PATH+"motor");
        plt_assert(ctl.is_open());
        ctl<<"3";
        for(i=0;i<MOTOR_NUM;i++){
            j=layoutMap[i];
            if(dirMap[i]){
                dir=(mParam[j].status==MOTOR_STATUS_POSITIVE?1:0);
            }else{
            dir=(mParam[j].status==MOTOR_STATUS_POSITIVE?0:1);
            }
            ctl<<":"<<i+1<<":"<<dir;
        //std::cout<<"dir: "<<i+1<<","<<dir<<endl;
        }
        ctl<<std::flush;
        return ctl.fail()?-1:0;


    }
    int MaccumMotor::setSpeed(bool stop)
    {
        static int layoutMap[]={0,1,3,2};
        int speed,i,j;

        ofstream ctl(PROC_PATH+"motor");
        plt_assert(ctl.is_open());
        ctl<<"5";
        for(i=0;i<MOTOR_NUM;i++){
            if(!stop){
                j=layoutMap[i];
                if(!MOTOR_SPEED_ZERO(mParam[j].speed)){
                    speed=(int)(mParam[j].speed*E);
                }else{
                    speed=0;
                }
                if(speed>=100){
                    speed=100;
                }
                if(speed<0){
                    speed=0;
                }
            }else{
            speed=0;
            }
            ctl<<":"<<i+1<<":"<<speed;
            //std::cout<<"speed: "<<i+1<<","<<speed<<endl;
        }
        ctl<<std::flush;
        return ctl.fail()?-1:0;

    }

    int MaccumMotor::drive(vector<int> vDir, vector<int> vSpeed)
    {
        if (vDir.size() != MOTOR_NUM &&
            vSpeed.size() !=MOTOR_NUM){
            return -1;
        }
        {
            ofstream ctl(PROC_PATH+"motor");
            plt_assert(ctl.is_open());
            ctl<<"3";
            for(int i=0;i<vDir.size();i++){
                ctl<<":"<<i+1<<":"<<vDir[i];
                std::cout<<"dir2: "<<i+1<<","<<vDir[i]<<endl;
            }
            ctl<<std::flush;
        }

        {
            int speed;
            ofstream ctl(PROC_PATH+"motor");
            plt_assert(ctl.is_open());
            ctl<<"5";
            for(int i=0;i<vSpeed.size();i++){
                speed = vSpeed[i];
                if(vSpeed[i]>=100){
                    speed=100;
                }
                // if(speed<0){
                //     speed=0;
                // }
                ctl<<":"<<i+1<<":"<<speed;
                std::cout<<"speed2 : "<<i+1<<","<<speed<<endl;
            }
            ctl<<std::flush;
        }

        return 0;
    }

    int MaccumMotor::flush()
    {
        int i;
        bool dirChange=false;

        for(i=0;i<MOTOR_NUM;i++){
            if(mDirChanged[i]==1){
                break;
            }
        }

        //std::cout<<"flush: "<<i<<endl;
        if(i!=MOTOR_NUM){//if direction changed
            if(setSpeed(true)<0){//set speed to zero
                return -1;
            }
            usleep(MOROR_SET_DIR_DELAY*1000);
            if(setDir()<0){
                return -1;
            }
            dirChange=true;
        }
        for(i=0;i<MOTOR_NUM&&!dirChange;i++){
            if(mSpeedChanged[i]==1){
                break;
            }
        }
        if(i!=MOTOR_NUM){
            if(setSpeed(false)<0){
                return -1;
            }
        }
        return 0;
    }


    MaccumMotorFS8003::MaccumMotorFS8003()
    {
        DeviceDefaultConfig cfg;
        MOROR_SET_DIR_DELAY=cfg.getMotorSetDirDelay();

        vector<float> vReverseSpeed = {196.17,188.92,175.42,158.5,139.5,
                            120.75,103.33,86.83,72.75,60.5,49.5,
                            39.5,30,19.33,7.92,3.17,0,0,0,0,0,0,0,0,0,0};


        vector<float> vForwardSpeed = {196.42,185.25,177.25,169.33,161.25,
                                153.17,145.33,137.17,129,120.92,112.83,
                                104.58,96.58,88.5,80.5,72.08,63.83,55.75,
                                47.67,39.5,31.5,23.5,15.5,7.67,0,0};


        int nSpan = 4;
        for (int i=0; i<vReverseSpeed.size()-1; i++){
            for (int j=0; j<nSpan; j++){
                m_vRSpeed.push_back(vReverseSpeed[i]-(vReverseSpeed[i]-vReverseSpeed[i+1])*j/nSpan);
            }
        }
        for (int i=0; i<vForwardSpeed.size()-1; i++){
            for (int j=0; j<nSpan; j++){
                m_vFSpeed.push_back(vForwardSpeed[i]-(vForwardSpeed[i]-vForwardSpeed[i+1])*j/nSpan);
            }
        }
        //PLOG_INFO(MOTOR_TAG,"Motor Change dir delay=%d\n",MOROR_SET_DIR_DELAY);
    }
    MaccumMotorFS8003::~MaccumMotorFS8003()
    {

    }
    int MaccumMotorFS8003::powerOn()
    {
        return 0;
    }
    int MaccumMotorFS8003::powerOff()
    {
        return 0;
    }

    int MaccumMotorFS8003::setParam(int idx,float speed,MotorStatus status)
    {
        if(idx<0||idx>=MOTOR_NUM){
            pset_errno(PEPAR);
            return -1;
        }

        //std::cout<<"setParam: "<<idx<<","<<speed<<","<<status<<endl;
        mSpeedChanged[idx]=(!MOTOR_SPEED_ZERO(speed) && MOTOR_SPEED_EQUAL(mParam[idx].speed,speed))?0:1;
        if(mSpeedChanged[idx]){
            mParam[idx].speed=speed;
        }
        //mDirChanged[idx]=(mParam[idx].status==status||MOTOR_SPEED_ZERO(speed))?0:1;//if speed == 0 ,do not change dir
        mDirChanged[idx]=mParam[idx].status!=status?1:0;//if speed == 0 ,do not change dir
        if(mDirChanged[idx]){
            mParam[idx].status=status;
        }
        if (MOTOR_SPEED_ZERO(speed)){
            mParam[idx].status=MOTOR_STATUS_MAX;
        }
        return 0;
    }

    /*
    High Level motor layout is
                         front
                2               0
left                                    right
                3               1
                     back
    Driver Level layout is
                             front
                1               0(r)
left                                    right
                2               3(r)
                     back
index map is:[0,2,1,3]
dir map [1,0,0,1]
    */
    int MaccumMotorFS8003::setDir()
    {
        int dir,i;

        string motorCtl[] = {"motor0_ctl","motor1_ctl","motor2_ctl","motor3_ctl"};
        for(i=0;i<MOTOR_NUM;i++){
            ofstream ctl(PROC_PATH+motorCtl[i]);
            plt_assert(ctl.is_open());
            if (mParam[i].status!=MOTOR_STATUS_POSITIVE){
                ctl<<"0";
                //std::cout<<"dir0: "<<i+1<<"  0"<<endl;
            }else{
                ctl<<"1";
                //std::cout<<"dir1: "<<i+1<<"  1"<<endl;
            }
            ctl<<std::flush;
        }

        return 0;

    }


    void MaccumMotorFS8003::convertSpeed(float& x, float& y)
    {
        if (y>0){
            if (y<0.05){
                y/=1.3;
            }else if (y<0.06){
                y/=1.09;
            }else   if (y<=0.07){
                y/=0.93;
            }else   if (y<=0.08){
                y/=0.83;
            }else if (y<=0.09){
                y/=0.96;
            }else   if (y<=0.13){
                y/=0.85;
            }else if (y<=0.17){
                y/=0.87;
            }else if (y<=0.2){
                y/=0.9;
            }else if (y<=0.47){
                y/=0.93;
            }
        }else {
            if (abs(y)<=0.06){
                y/=1.54;
            }else  if (abs(y)<=0.07){
                y/=1.43;
            }else  if (abs(y)<=0.08){
                y/=1.23;
            }else  if (abs(y)<=0.09){
                y/=1.13;
            }else  if (abs(y)<=0.1){
                y/=1;
            }else if (abs(y)<=0.17){
                y/=0.92;
            }else if (abs(y)<=0.24){
                y/=0.95;
            }else if (abs(y)<=0.28){
                y/=0.97;
            }else if (abs(y)<=0.32){
               y/=1;
            }else{
                y/=1.04;
            }
        }

        if (x<0){
            if (abs(x)<=0.04){

            }else if (abs(x)<=0.05){
                x/=0.8;
            }else if (abs(x)<=0.06){
                x/=0.72;
            }else if (abs(x)<=0.07){
               // x/=0.65;
                x/=0.72;
            }else if (abs(x)<=0.08){
                //x/=0.55;
                x/=0.66;
            }else if (abs(x)<=0.09){
                x/=0.99;
            }else if (abs(x)<=0.1){
                x/=0.65;
            }else if (abs(x)<=0.12){
                x/=0.68;
            }else if (abs(x)<=0.15){
                x/=0.7;
            }else if (abs(x)<=0.2){
                x/=0.77;
            }else if (abs(x)<=0.24){
                x/=0.8;
            }else if (abs(x)<=0.28){
                x/=0.8;
            }else if (abs(x)<=0.34){
                x/=0.82;
            }else if (abs(x)<=0.41){
                x/=0.85;
            }else{
                x/=0.82;
            }
        }else{
            if (x<=0.04){

            }else  if (x<=0.05){
                x/=0.65;
            }else if (x<=0.06){
                x/=0.52;
            }else if (x<=0.07){
                //x/=0.46;
                x/=0.69;
            }else  if (x<=0.08){
                x/=0.69;
            }else  if (x<=0.09){
                x/=0.93;
            }else  if (x<=0.1){
                x/=0.65;
            }else if (x<=0.11){
                x/=0.68;
            }else if (x<=0.12){
                x/=0.68;
            }else if (x<=0.17){
                x/=0.7;
            }else if (x<=0.2){
                x/=0.74;
            }else if (x<=0.24){
                x/=0.75;
            }else if (x<=0.29){
                x/=0.76;
            }else if (x<=0.34){
                x/=0.78;
            }else{
                x/=0.8;
            }
        }
    }

    void MaccumMotorFS8003::convertSpeedTrackModel(float& x, float& y)
    {
        ///< for tunning
        // DeviceDefaultConfig cfg;
        // float tune_number = cfg.getTuneNumber();
        ///< put this pieces of code under range want to tune.
        // PLOG_INFO(MOTOR_TAG, "[Tunning] [x: %f, y: %f], k = %f", x, y, tune_number);
        if (y>0){
            if (y<=0.071){
                // y /= tune_number;
                y/=0.68;
            }else   if (y<=0.085){
                // y /= tune_number;
                y/=0.71;
            }else if (y<=0.095){
                // y /= tune_number;
                y/=0.73;
            }else   if (y<=0.13){
                // y /= tune_number;
                y/=0.75;
            }else if (y<=0.17){
                // y /= tune_number;
                y/=0.82;
            }else if (y<=0.22){
                // y /= tune_number;
                y/=0.84;
            }else if (y<=0.47){
                // y /= tune_number;
                y/=0.86;
            }
        }else {
            if (abs(y)<=0.071){
                // y = 0.07;
                // y /= tune_number;
                y/=0.71;
            }else  if (abs(y)<=0.085){
                // y /= tune_number;
                y/=0.78;
            }else  if (abs(y)<=0.095){
                // y /= tune_number;
                y/=0.82;
            }else  if (abs(y)<=0.12){
                // y /= tune_number;
                y/=0.83;
            }else if (abs(y)<=0.17){
                // y /= tune_number;
                y/=0.90;
            }else if (abs(y)<=0.24){
                // y /= tune_number;
                y/=1.02;
            }else if (abs(y)<=0.28){
                // y /= tune_number;
                y/=1.05;
            }else if (abs(y)<=0.32){
                // y /= tune_number;
                y/=1.1;
            }else{
                // y /= tune_number;
                y/=1.18;
            }
        }
        x = 0;
    }


    // float MaccumMotorFS8003::convertSpeed(float speed)
    // {
    //     if (speed>0){
    //         if (speed<=0.13){
    //             return speed/0.85;
    //         }else if (speed<=0.17){
    //             return speed/0.87;
    //         }else if (speed<=0.2){
    //             return speed/0.9;
    //         }else if (speed<=0.47){
    //             return speed/0.93;
    //         }
    //     }else {
    //         if (speed<=0.1){
    //             return speed;
    //         }else if (speed<=0.17){
    //             return speed/0.92;
    //         }else if (speed<=0.24){
    //             return speed/0.95;
    //         }else if (speed<=0.28){
    //             return speed/0.97;
    //         }else if (speed<=0.32){
    //             return speed;
    //         }else{
    //             return speed/1.04;
    //         }
    //     }

    //     return speed;
    // }

#if 1
    int MaccumMotorFS8003::setSpeed(bool stop)
    {
        const int MAX_SPEED = 100;
        //const int MIN_POSSPEED = 18;
        int MIN_POSSPEED, MIN_NEGSPEED;
        DeviceDefaultConfig cfg;
        // float matching_factor = cfg.getMatchingFactor();
        if (PltConfig::getInstance()->isTrackModel()){  /// Track model ...
            MIN_POSSPEED = 17;
            MIN_NEGSPEED = 62;
            // MIN_POSSPEED = cfg.getTuneNumber();
            // MIN_NEGSPEED = cfg.getTuneNumber2();
        } else {
            MIN_POSSPEED = 16;
            MIN_NEGSPEED = 58;
        }
        // const int MIN_POSSPEED = 16;
        string motorCtl[] = {"motor0_ctl","motor1_ctl","motor2_ctl","motor3_ctl"};
        //calc match speed
        int speedArr[MOTOR_NUM] = {0};
        for(int i=0;i<MOTOR_NUM;i++){
            if(!stop){
                if(!MOTOR_SPEED_ZERO(mParam[i].speed)){
                    speedArr[i]=(int)(mParam[i].speed*E);
                    speedArr[i] = (speedArr[i]*MAX_SPEED)/100;
                    if(speedArr[i]>=MAX_SPEED){
                        speedArr[i]=MAX_SPEED;
                    }else if (speedArr[i] < MIN_POSSPEED){
                        speedArr[i]=MIN_POSSPEED;
                    }
                }
            }
        }
        for(int i=0;i<MOTOR_NUM;i++){
            if (MOTOR_STATUS_POSITIVE != mParam[i].status &&
                speedArr[i]>=MIN_POSSPEED){
                speedArr[i] = (speedArr[i]-MIN_POSSPEED)/2+MIN_NEGSPEED + MATCHING_FACTOR;
            }
        }
        std::cout << std::endl;

        for(int i=0;i<MOTOR_NUM;i++){
            ofstream ctl(PROC_PATH+motorCtl[i]);
            plt_assert(ctl.is_open());
            ctl<<"2:"<< speedArr[i];
            ctl<<std::flush;
        }
        return 0;
    }
#else

    int MaccumMotorFS8003::setSpeed(bool stop)
    {
        const int MAX_SPEED = 100;
        const int MIN_POSSPEED = 18;
        string motorCtl[] = {"motor0_ctl","motor1_ctl","motor2_ctl","motor3_ctl"};
        //calc match speed
        int speedArr[MOTOR_NUM] = {0};
        for(int i=0;i<MOTOR_NUM;i++){
            if(!stop){
                if(!MOTOR_SPEED_ZERO(mParam[i].speed)){
                    speedArr[i]=(int)(mParam[i].speed*E);
                    speedArr[i] = (speedArr[i]*MAX_SPEED)/100;
                    if(speedArr[i]>=MAX_SPEED){
                        speedArr[i]=MAX_SPEED;
                    }else if (speedArr[i] < MIN_POSSPEED){
                        speedArr[i]=MIN_POSSPEED;
                    }
                }
            }

            std::cout<<"i="<<i<<","<<speedArr[i]<<std::endl;
        }

        const int MIN_NEGSPEED = 59;
        //const int MIN_NEGSPEED = 58;
         for(int k=0;k<MOTOR_NUM;k++){
            if (MOTOR_STATUS_POSITIVE != mParam[k].status &&
                  speedArr[k]>=MIN_POSSPEED){
                int idx = MAX_SPEED-speedArr[k];
                for (int j=0; j<m_vRSpeed.size()-1; j++){
                    if (m_vFSpeed[idx]>m_vRSpeed[j+1]){
                        speedArr[k] = m_vRSpeed.size()-j;
                        std::cout<<"k="<<k<<","<<speedArr[k]<<","<<"idx,j="<<idx<<","<<100-j<<", "<<m_vFSpeed[idx]<<","<<m_vRSpeed[j]<<std::endl;
                        break;
                    }
                }
            }
        }



        for(int i=0;i<MOTOR_NUM;i++){
            ofstream ctl(PROC_PATH+motorCtl[i]);
            plt_assert(ctl.is_open());
            ctl<<"2:"<< speedArr[i];
            ctl<<std::flush;
            //std::cout<<"FS8003 speed: "<<i+1<<","<<mParam[i].speed<<" ,"<< speedArr[i]<<endl;
        }
        return 0;
    }
#endif

    int MaccumMotorFS8003::drive(vector<int> vDir, vector<int> vSpeed)
    {
        const int MAX_SPEED = 100;
        const int MIN_POSSPEED = 18;
        string motorCtl[] = {"motor0_ctl","motor1_ctl","motor2_ctl","motor3_ctl"};

        for(int i=0;i<vSpeed.size();i++){
            if (vSpeed[i]>0){
                 if(vSpeed[i]>=MAX_SPEED){
                        vSpeed[i]=MAX_SPEED;
                    }else if (vSpeed[i] < MIN_POSSPEED){
                        vSpeed[i]=MIN_POSSPEED;
                    }
            }
        }

        const int MIN_NEGSPEED = 59;
         for(int i=0;i<vDir.size();i++){
            if (0 == vDir[i] &&
                  vSpeed[i]>=MIN_POSSPEED){
                vSpeed[i] = (vSpeed[i]-MIN_POSSPEED)/2+MIN_NEGSPEED;
            }

            std::cout<<"FS8003 : "<<i+1<<","<< vDir[i]<<" ,"<< vSpeed[i]<<endl;
        }

        for(int i=0;i<vSpeed.size();i++){
            ofstream ctl(PROC_PATH+motorCtl[i]);
            plt_assert(ctl.is_open());
            if (vDir[i]>0){
                ctl<<"1";
            }else{
                ctl<<"0";
            }
            ctl<<std::flush;
            ctl<<"2:"<< vSpeed[i];
            ctl<<std::flush;
        }
        return 0;
    }


    int MaccumMotorFS8003::driveF8003(int idx, float speed)
    {
        if (idx<0 || idx>3){
            return -1;
        }
        string motorCtl[] = {"motor0_ctl","motor1_ctl","motor2_ctl","motor3_ctl"};
        int i=idx;
        int dir = 0;
        static int vPrevDir[] ={-1,-1,-1,-1};
        std::cout<<"driveF8003 i= "<<i<<endl;
        ofstream ctl(PROC_PATH+motorCtl[i]);
        plt_assert(ctl.is_open());
        if (speed>0){
            ctl<<"0";
            std::cout<<"driveF8003 dir 0"<<endl;
        }else{
            dir = 1;
            ctl<<"1";
            std::cout<<"driveF8003 dir 1"<<endl;
        }
        ctl<<std::flush;
        if (abs(speed) < 0.001){
            ctl<<"3";
            std::cout<<"driveF8003 stop : "<<endl;
            vPrevDir[i] = -1;
        }else{
            int nSpeed = abs(speed);
            if(nSpeed>=24){
                    nSpeed=24;
            }
            if (dir != vPrevDir[i]){
                ctl<<"2:"<<0;
                ctl<<std::flush;
                usleep(100*1000);
                int nTmp = 20;
                ctl<<"2:"<<nTmp;
                ctl<<std::flush;
                usleep(20*1000);
                std::cout<<"driveF8003 speed0 : "<<speed<<" "<<nTmp<<endl;
            }
            ctl<<"2:"<<nSpeed;
            std::cout<<"driveF8003 speed : "<<speed<<" "<<nSpeed<<endl;
            vPrevDir[i] = dir;
        }
        ctl<<std::flush;
        return 0;
    }

    int MaccumMotorFS8003::flush()
    {
        int i;
        bool dirChange=false;

        for(i=0;i<MOTOR_NUM;i++){
            if(mDirChanged[i]==1){
                break;
            }
        }

        //std::cout<<"flush: "<<i<<endl;
        if(i!=MOTOR_NUM){//if direction changed
            if(setSpeed(true)<0){//set speed to zero
                return -1;
            }
            usleep(MOROR_SET_DIR_DELAY*1000);
            if(setDir()<0){
                return -1;
            }
            dirChange=true;
        }
        for(i=0;i<MOTOR_NUM&&!dirChange;i++){
            if(mSpeedChanged[i]==1){
                break;
            }
        }
        if(i!=MOTOR_NUM){
            if(setSpeed(false)<0){
                return -1;
            }
        }
        return 0;
    }

    TTYMotor::TTYMotor(const string& path,int baudrate)
    {
        plt_assert(change_file_mode(path.c_str(),"666")==0);

        mFd=open(path.c_str(),O_RDWR|O_NOCTTY|O_NDELAY);
        plt_assert(mFd>=0);

        plt_assert(set_serial_attrib(mFd,baudrate,8,"1",'N',0,0)==0);
    }
    TTYMotor::~TTYMotor()
    {
        if(mFd>=0){
            close(mFd);
        }
    }
    inline int TTYMotor::writeMsg(const char* cmd)
    {
        int ret;
        int len=strlen(cmd);
        if((ret=write(mFd,cmd,len))!=len){
            PLOG_ERROR(MOTOR_TAG,"Write [%s] fail,len=%d\n",cmd,ret);
        }
        return 0;
    }

    TTYMotor1::TTYMotor1(const string& path,int baudrate):
     TTYMotor(path,baudrate)
    {
    }
    TTYMotor1::~TTYMotor1()
    {

    }
    int TTYMotor1::powerOn()
    {
        const char* cmd="PO\n";
        return writeMsg(cmd);
    }
    int TTYMotor1::powerOff()
    {
        const char* cmd="PF\n";
        return writeMsg(cmd);
    }
    int TTYMotor1::flush()
    {
        const char* cmd="F\n";
        return writeMsg(cmd);
    }

    int TTYMotor1::setParam(int idx,float speed,MotorStatus status)
    {
        char buf[32];
        int  dir=(status==MOTOR_STATUS_POSITIVE?1:0);
        snprintf(buf,sizeof(buf),"S %d %d %f\n",idx,dir,speed);
        return writeMsg(buf);
    }
    TTYMotor2::TTYMotor2(const string& path,int baudrate):
    TTYMotor(path,baudrate)
    {
    }
    TTYMotor2::~TTYMotor2()
    {

    }
    int TTYMotor2::powerOn()
    {
        return 0;
    }
    int TTYMotor2::powerOff()
    {
        const char* cmd="OFF\n";
        return writeMsg(cmd);
    }
    int TTYMotor2::flush()
    {
        char buf[32];
        float speed=0;

        for(int i=0;i<MOTOR_NUM;i++){
            speed+=mParam[i].speed;
        }
        speed/=MOTOR_NUM;
        int idx=snprintf(buf,sizeof(buf),"S%d",(int)(speed*E));
        for(int i=0;i<MOTOR_NUM;i++){
            idx+=snprintf(buf+idx,sizeof(buf)-idx,"D%d",mParam[i].status==MOTOR_STATUS_POSITIVE?1:0);
        }
        snprintf(buf+idx,sizeof(buf)-idx,"\n");
        return writeMsg(buf);
    }

    int TTYMotor2::setParam(int idx,float speed,MotorStatus status)
    {
        return 0;
    }
    Motor* createMotor()
    {
#if defined(USE_DEFALUT_MOTOR)
        if (PltConfig::getInstance()->isChinaMotor()){
            return new MaccumMotorFS8003();
        }

        return new MaccumMotor();
#elif defined(USE_PLUGIN_MOTOR)
        std::string serialPort;
        PltConfig::getInstance()->getMotorPort(serialPort);
#if 0
        return new TTYMotor1(serialPort,B115200);
#else
        return new TTYMotor2(serialPort,B38400);
#endif
#else
        return nullptr;
#endif
    }
}