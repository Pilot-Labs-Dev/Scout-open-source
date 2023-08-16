#ifndef __ROLLER_EYE_MOTOR__H_
#define __ROLLER_EYE_MOTOR__H_

#include<memory>

using namespace std;
namespace roller_eye{
    enum MotorStatus{
        MOTOR_STATUS_POSITIVE,
        MOTOR_STATUS_NEGTIVE,
        MOTOR_STATUS_LOCK,
        MOTOR_STATUS_MAX
    };
#define MOTOR_NUM   4

#define USE_DEFALUT_MOTOR
//#define USE_PLUGIN_MOTOR

#if defined(USE_DEFALUT_MOTOR)
#define MACC_MOTOR_MAX_RPM          (180.0) //unit:rpm
#define MACC_CAR_WIDTH         8.0e-2   // unit: m
#define MACC_CAR_HEIGHT        5.3e-2
#define MACC_ROLL_DIAMETER   5.0e-2
#elif defined(USE_PLUGIN_MOTOR)
#define MACC_MOTOR_MAX_RPM          (180.0) //unit:rpm
#define MACC_CAR_WIDTH         0.14   // unit: m
#define MACC_CAR_HEIGHT        0.16
#define MACC_ROLL_DIAMETER   8.0e-2
#else
#error "motor type error"
#endif

#define MACC_PI                                                 (3.1415926)
#define MACC_MOTOR_MAX_F                    (MACC_MOTOR_MAX_RPM/60.0)
#define MACC_MAX_SPEED_X                      (MACC_MOTOR_MAX_F*MACC_PI*MACC_ROLL_DIAMETER)        //0.47(defalut motor)
#define MACC_MAX_SPEED_Y                      (MACC_MOTOR_MAX_F*MACC_PI*MACC_ROLL_DIAMETER)
#define MACC_MAX_SPEED_W                     ((MACC_MOTOR_MAX_F*MACC_PI*MACC_ROLL_DIAMETER)/(MACC_CAR_WIDTH/2+MACC_CAR_HEIGHT/2)) //7.08(defalut motor)

 enum class MotorDriverType{STSPIN240, FS8003};
   class Motor{
    public:
        Motor();
        virtual ~Motor();

        virtual MotorDriverType GetDriverType(){return MotorDriverType::STSPIN240;}

        virtual int setParam(int idx,float speed,MotorStatus status);
        virtual int driveF8003(int idx, float speed){}
        int powerON();
        int powerOFF();
        virtual int flush()=0;

        virtual int drive(vector<int> vDir, vector<int> vSpeed){}
        virtual void convertSpeed(float& x, float& y){}
        virtual void convertSpeedTrackModel(float& x, float& y){}
    protected:
        virtual int powerOn()=0;
        virtual int powerOff()=0;
        void reset();
        struct Param{
            float speed;
            MotorStatus status;
        } ;
        Param mParam[MOTOR_NUM];
        int mSpeedChanged[MOTOR_NUM];
        int mDirChanged[MOTOR_NUM];
        int MOROR_SET_DIR_DELAY;
        constexpr static  float E=100.0/(MACC_MOTOR_MAX_F);
   };

   class MaccumMotor:public Motor{
    public:
        MaccumMotor();
        ~MaccumMotor();

        int flush();
        int drive(vector<int> vDir, vector<int> vSpeed);
        MotorDriverType GetDriverType(){return MotorDriverType::STSPIN240;}
    private:
        int powerOn();
        int powerOff();
        int setDir();
        int setSpeed(bool stop);
   };

    class MaccumMotorFS8003:public Motor{
    public:
        MaccumMotorFS8003();
        ~MaccumMotorFS8003();

        int flush();
        int drive(vector<int> vDir, vector<int> vSpeed);
        void convertSpeed(float& x, float& y);
        void convertSpeedTrackModel(float& x, float& y);
        virtual int driveF8003(int idx, float speed);
        MotorDriverType GetDriverType(){return MotorDriverType::FS8003;}
    private:
        int powerOn();
        int powerOff();
        int setDir();
        int setSpeed(bool stop);
        int setParam(int idx,float speed,MotorStatus status);
        vector<float> m_vRSpeed;
        vector<float> m_vFSpeed;
   };
   class TTYMotor:public Motor{
    public:
        TTYMotor(const string& path,int baudrate);
        ~TTYMotor();

    protected:
        inline int writeMsg(const char* cmd);
    private:
        int mFd;
   };

    class TTYMotor1:public TTYMotor{
    public:
        TTYMotor1(const string& path,int baudrate);
        ~TTYMotor1();

        int flush();
    private:
        int powerOn();
        int powerOff();
        int setParam(int idx,float speed,MotorStatus status);
   };
   class TTYMotor2:public TTYMotor{
    public:
        TTYMotor2(const string& path,int baudrate);
        ~TTYMotor2();

        int flush();
    private:
        int powerOn();
        int powerOff();
        int setParam(int idx,float speed,MotorStatus status);
   };
   Motor* createMotor();
}
#endif