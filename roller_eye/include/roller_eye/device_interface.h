#ifndef __ROLLER_EYE_DEVICE_INTERFACE__H__
#define __ROLLER_EYE_DEVICE_INTERFACE__H__

namespace roller_eye{
int open_ir_cut(void);
int close_ir_cut(void);

int open_ir_led(int brightness);
int close_ir_led();

int set_power_led_status(int id,bool status);
}
#endif