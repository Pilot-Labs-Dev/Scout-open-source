#include<fstream>
#include<string>
#include<sstream>
#include"device_interface.h"

namespace roller_eye{

static const std::string DRIVER_PATH="/proc/driver/";
static const std::string IR_CUT_PATH=DRIVER_PATH+"ir_cut";
static const std::string IR_LED_PATH=DRIVER_PATH+"light";
static const std::string POWER_LED=DRIVER_PATH+"gpioctl";

static int write_ctl_msg(const std::string& path,const std::string& msg)
{
    std::ofstream file(path);
    file<<msg<<std::flush;
    return file.good()?0:-1;
}
int open_ir_cut(void)
{
    return write_ctl_msg(IR_CUT_PATH,"1");
}
int close_ir_cut(void)
{
    return write_ctl_msg(IR_CUT_PATH,"0");
}

int open_ir_led(int brightness)
{
    return write_ctl_msg(IR_LED_PATH,std::to_string(brightness));
}
int close_ir_led()
{
    return write_ctl_msg(IR_LED_PATH,"0");
}
int set_power_led_status(int id,bool status)
{
    if(id<0||id>=4){
        return -1;
    }
    id+=2;
    std::stringstream ss;
    ss<<"gpio0"<<id<<"_"<<(status?1:0);
    return write_ctl_msg(POWER_LED,ss.str());
}
}