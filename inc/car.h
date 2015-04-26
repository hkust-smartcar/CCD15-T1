///*
// * car.h
// *
// *  Created on: 3 Mar, 2015
// *      Author: Howard
// */
//
//#include <libbase/k60/mcg.h>
//#include <libsc/led.h>
//#include <libsc/system.h>
//#include <libsc/alternate_motor.h>
//#include <libsc/tower_pro_mg995.h>
//#include <libsc/mpu6050.h>
//#include <libsc/encoder.h>
//#include <libsc/dir_motor.h>
//#include <libsc/mma8451q.h>
//#include <libsc/device_h/mma8451q.h>
//#include <cstdio>
//#include <math.h>
//#include <libsc/tsl1401cl.h>
//#include "libsc/st7735r.h"
//#include <libsc/lcd_console.h>
//#include <libsc/lcd_typewriter.h>
//#include <libbase/k60/adc.h>
//#include <libsc/joystick.h>
//#include <libsc/dir_encoder.h>
//#include <libutil/string.h>
//#include <libsc/k60/jy_mcu_bt_106.h>
//#include <libsc/ab_encoder.h>
//#include <libutil/remote_var_manager.h>
//#include <kalman.h>
//#include <libsc/button.h>
//
//#ifndef INC_CAR_H_
//#define INC_CAR_H_
//
//using namespace libbase;
//using namespace libsc;
//using namespace libbase::k60;
//using namespace libsc::k60;
//using namespace libutil;
//
//
//#include <libbase/k60/mcg.h>
//#include <libsc/led.h>
//#include <libsc/system.h>
//#include <libsc/alternate_motor.h>
//#include <libsc/tower_pro_mg995.h>
//#include <libsc/mpu6050.h>
//#include <libsc/encoder.h>
//#include <libsc/dir_motor.h>
//#include <libsc/mma8451q.h>
//#include <libsc/device_h/mma8451q.h>
//#include <cstdio>
//#include <math.h>
//#include <libsc/tsl1401cl.h>
//#include "libsc/st7735r.h"
//#include <libsc/lcd_console.h>
//#include <libsc/lcd_typewriter.h>
//#include <libbase/k60/adc.h>
//#include <libsc/joystick.h>
//#include <libsc/dir_encoder.h>
//#include <libutil/string.h>
//#include <libsc/k60/jy_mcu_bt_106.h>
//#include <libsc/ab_encoder.h>
//#include <libutil/remote_var_manager.h>
//#include <kalman.h>
//#include <libsc/button.h>
//
//using namespace libsc;
//using namespace libsc::k60;
//using namespace libutil;
//
//class Car{
//public:
//	Car();
//
//private:
//	JyMcuBt106 bt;
//	Mpu6050 mpu6050;
//	Joystick joy;
//	AlternateMotor motor_r;
//	AlternateMotor motor_l;
//	AbEncoder encoder_l;
//	AbEncoder encoder_r;
//	Tsl1401cl ccd;
//	Adc PowerTest;
//	St7735r lcd;
//	Button button0;
//	Button button1;
//	libsc::Led led0;
//	libsc::Led led1;
//	double kalman_value[2] = {0.1, -1.0};
//	Kalman acc;
//
//
//};
//
//
//
//
//
//#endif /* INC_CAR_H_ */
