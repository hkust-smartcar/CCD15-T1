///*
// * main.cpp
// *
// *  Created on: 26 Dec, 2014
// *      Author: Howard
// */
//
//#include <libbase/k60/mcg.h>
//#include <libsc/led.h>
//#include <libsc/system.h>
//#include <libsc/ftdi_ft232r.h>
//#include <libsc/alternate_motor.h>
//#include <libsc/tower_pro_mg995.h>
//#include <libsc/mpu6050.h>
//#include <libsc/encoder.h>
//#include <libsc/dir_motor.h>
//#include <libsc/mma8451q.h>
//#include <libsc/device_h/mma8451q.h>
//#include <cstdio>
//#include <libsc/linear_ccd.h>
//#include "libsc/st7735r.h"
//#include <libsc/lcd_console.h>
//#include <libsc/lcd_typewriter.h>
//#include <libbase/k60/adc.h>
//#include <libsc/joystick.h>
//#include <libsc/dir_encoder.h>
//#include <libutil/string.h>
//#include <libutil/kalman_filter.h>
//#include <kalman.h>
//#include "../inc/MyVarManager.h"
//
//#define BLACK           0x0000
//#define BLUE            0x001F
//#define RED             0xF800
//#define GREEN           0x07E0
//#define CYAN            0x07FF
//#define MAGENTA         0xF81F
//#define YELLOW          0xFFE0
//#define WHITE           0xFFFF
//
//float ideal_count_Kd = 0;
//float ideal_count_Kp = 0;
//float error_kd = 0;
//float ic_Kp = 7.5;
//float ic_Ki = 0;
//float ic_Kd = 0.68;
//float gyro_Ki = 0;
//float encoder_Kp = 35;
//float encoder_Ki = 0;
//float encoder_Kd = 0.03;
//
//int32_t last_ideal_count = 0;
//int32_t ideal_count = 0;
//float original_angle = 0;
//Byte i=0;
//
//uint32_t pt1 = 0;
//uint32_t pt2 = 0;
//uint32_t pt3 = 0;
//uint32_t pt4 = 0;
//uint32_t pt5 = 0;
//
//Byte yo = 0;                      //to organize the sequence of code
//
//
//
//class Wheel_l{
//	DirEncoder encoder_l;
//	int32_t count_l = 0;
//	AlternateMotor motor_l;
//	int32_t last_il_encoder_error = 0;
//	int32_t il_encoder_error = 0;
//	int32_t il_encoder_error_change = 0;
//	int32_t il_encoder_errorsum = 0;
//	int32_t speed_l = 0;
//	int32_t old_speed_l = 0;
//} wheel_l;
//
//
//class Wheel_r{
//public:
//	DirEncoder encoder_r;
//	int32_t count_r = 0;
//	AlternateMotor motor_r;
//	int32_t last_ir_encoder_error = 0;
//	int32_t ir_encoder_error = 0;
//	int32_t ir_encoder_error_change = 0;
//	int32_t ir_encoder_errorsum = 0;
//	int32_t speed_r = 0;
//	int32_t old_speed_r = 0;
//
//} wheel_r;
//
//class Common_Para{
//public:
//	int last_sign = 0;
//	int sign = 0;
//	float old_ratio = 0.75;
//}  common;
//
//class Gyro_Accel{
//public:
//	Mpu6050 mpu6050;
//	std::array<float, 3>accel;
//	std::array<float, 3>angle;
//	std::array<float, 3>omega;
//	double accel_angle = original_angle;
//	double last_gyro_angle = original_angle;
//	double gyro_angle = original_angle;
//	double last_accel_angle = original_angle;
//	double output_angle;
//	float trust_gyro = 1;
//	float trust_accel = 1 - trust_gyro;
//	double last_angle_error = 0;
//	double now_angle_error = 0;
//	double angle_error_change;
//} gyro_accel;
//
//class Lcd{
//	uint16_t result;
//	char *buffer = new char[125]{0};
//	char *words = new char[125]{0};
//	uint16_t pixel1[2100];
//	uint16_t pixel_bg_colour[2100];
//	uint32_t avg = 0;
//	uint32_t all = 0;
//	std::array<uint16_t,LinearCcd::kSensorW> pixel;
//	float window = 5.0;
//	float window_avg = 0;
//	int state = 0;
//
//	const char *screen1 = "Interstellar\n\n>Sensor State\n\n Balance Mode\n\n Come Back Mode\n\n Run Forest!!!";
//	const char *screen2 ="Interstellar\n"
//			"\n"
//			" Sensor State\n"
//			"\n"
//			">Balance Mode\n"
//			"\n"
//			" Come Back Mode\n"
//			"\n"
//			" Run Forest!!!";
//	char screen3[] = "Interstellar\n"
//			"\n"
//			" Sensor State\n"
//			"\n"
//			" Balance Mode\n"
//			"\n"
//			">Come Back Mode\n"
//			"\n"
//			" Run Forest!!!";
//	char screen4[] = "Interstellar\n"
//			"\n"
//			" Sensor State\n"
//			"\n"
//			" Balance Mode\n"
//			"\n"
//			" Come Back Mode\n"
//			"\n"
//			">Run Forest!!!";
//
//	char screen5[] = "Go motor!!!";
//} lcd;
//
//
//
//void Balance_function(Wheel_l &wheel_l, Wheel_r &wheel_r, float original_angle, int &last_ideal_count, Gyro_Accel &gyro_accel);
//void Follow_Encoder (Wheel_l &wheel_l, Wheel_r &wheel_r, Common_Para &common);
//
//char CCD;
//using namespace libsc;
//
//namespace libbase
//{
//namespace k60
//{
//
//Mcg::Config Mcg::GetMcgConfig()
//{
//	Mcg::Config config;
//	//Do Not change
//	config.external_oscillator_khz = 50000;
//	//Set MU clock to 100MHz
//	config.core_clock_khz = 180000;
//	return config;
//}
//
//}
//}
//
//
//void Follow_Encoder ();
//
//void Balance_function(Wheel_l &wheel_l, Wheel_r &wheel_r, float original_angle, int &last_ideal_count, Gyro_Accel &gyro_accel, Kalman kalman)
//{
//	wheel_r.encoder_r.Update();
//	wheel_l.encoder_l.Update();
//
//	gyro_accel.mpu6050.Update();
//
//	std::array<float, 3>accel;
//	std::array<float, 3>omega;
//
//	accel = gyro_accel.mpu6050.GetAccel();
//	omega = gyro_accel.mpu6050.GetOmega();
//
//	gyro_accel.last_accel_angle = gyro_accel.accel_angle;
//	gyro_accel.accel_angle = accel[0]*57.29578;
//	gyro_accel.accel_angle = 0.65*gyro_accel.last_accel_angle +0.35*gyro_accel.accel_angle;
//
//	gyro_accel.last_gyro_angle = gyro_accel.gyro_angle;
//	gyro_accel.gyro_angle += (-1) *omega[1]*0.005 + 0.01*(gyro_accel.accel_angle - gyro_accel.gyro_angle);
//	//			gyro_angle = accel_angle+(-1) *omega[0];
//	//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;
//
//
//	kalman.Filtering(&gyro_accel.output_angle, gyro_accel.gyro_angle, gyro_accel.accel_angle);
//
//	//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
//	//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
//	//	gyro_accel.output_angle = gyro_accel.trust_gyro*gyro_accel.gyro_angle +gyro_accel.trust_accel * gyro_accel.accel_angle;
//
//
//
//
//	gyro_accel.last_angle_error = gyro_accel.now_angle_error;
//	gyro_accel.now_angle_error = gyro_accel.output_angle - original_angle;
//	gyro_accel.angle_error_change = gyro_accel.now_angle_error - gyro_accel.last_angle_error;
//
//	last_ideal_count = ideal_count;
//	int temp_sign = 1;
//	if (gyro_accel.now_angle_error < 0)
//		temp_sign = -1;
//	ideal_count = (int32_t)(ic_Kp * temp_sign * gyro_accel.now_angle_error + ic_Kd * gyro_accel.angle_error_change);
//
//	ideal_count = (int32_t)(0.5 * last_ideal_count + 0.5 * ideal_count);
//}
//
//
//
//
//
//void Follow_Encoder (Wheel_l &wheel_l, Wheel_r &wheel_r, Common_Para &common)
//{
//	wheel_r.encoder_r.Update();
//	wheel_l.encoder_l.Update();
//
//	wheel_r.count_r = -1 * wheel_r.encoder_r.GetCount() / 3;
//	wheel_l.count_l = wheel_l.encoder_l.GetCount() / 3;
//
//	common.last_sign = common.sign;
//	if(ideal_count > 0){
//		common.sign = 1;
//	}
//	else if(ideal_count < 0){
//		common.sign = 0;
//	}
//	else if(ideal_count ==0){
//		common.sign = 2;
//	}
//
//
//	if(common.sign == 2){
//		wheel_l.motor_l.SetPower(0);
//		wheel_r.motor_r.SetPower(0);
//	}
//
//	else{
//
//		if(common.last_sign != common.sign){
//			wheel_l.motor_l.SetPower(0);
//			wheel_r.motor_r.SetPower(0);
//			wheel_l.motor_l.SetClockwise(common.sign);
//			wheel_r.motor_r.SetClockwise(common.sign);
//		}
//
//		wheel_r.last_ir_encoder_error = wheel_r.ir_encoder_error;
//		wheel_r.ir_encoder_error = ideal_count - wheel_r.count_r;
//		wheel_l.last_il_encoder_error = wheel_l.il_encoder_error;
//		wheel_l.il_encoder_error = ideal_count - wheel_l.count_l;
//
//
//		wheel_l.il_encoder_error_change = wheel_l.il_encoder_error -wheel_l.last_il_encoder_error;
//		wheel_r.ir_encoder_error_change = wheel_r.ir_encoder_error -wheel_r.last_ir_encoder_error;
//
//		wheel_r.old_speed_r = wheel_r.speed_r;
//		wheel_l.old_speed_l = wheel_l.speed_l;
//
//		wheel_r.ir_encoder_errorsum += wheel_r.ir_encoder_error * 0.003;
//		wheel_l.il_encoder_errorsum += wheel_l.il_encoder_error * 0.003;
//
//		wheel_r.speed_r = (int32_t)(wheel_r.ir_encoder_error * encoder_Kp
//				+ wheel_r.ir_encoder_error_change*encoder_Kd/0.003
//				+ wheel_r.ir_encoder_errorsum * encoder_Ki);
//		wheel_l.speed_l = (int32_t)(wheel_l.il_encoder_error * encoder_Kp
//				+ wheel_l.il_encoder_error_change*encoder_Kd/0.003
//				+ wheel_l.il_encoder_errorsum * encoder_Ki);
//		if(wheel_l.speed_l > 1000 && common.sign ==1){
//			wheel_l.speed_l = 1000;
//		}
//		else if(wheel_l.speed_l < 0 && common.sign ==1){
//			wheel_l.speed_l = 0;
//		}
//		else if(wheel_l.speed_l > 0 && common.sign ==0){
//			wheel_l.speed_l = 0;
//		}
//		else if(wheel_l.speed_l < -1000 && common.sign ==0){
//			wheel_l.speed_l = -1000;
//		}
//
//
//		if(wheel_r.speed_r > 1000 && common.sign ==1){
//			wheel_r.speed_r = 1000;
//		}
//		else if(wheel_r.speed_r < 0 && common.sign ==1){
//			wheel_r.speed_r = 0;
//		}
//		else if(wheel_r.speed_r > 0 && common.sign ==0){
//			wheel_r.speed_r = 0;
//		}
//		else if(wheel_r.speed_r < -1000 && common.sign ==0){
//			wheel_r.speed_r = -1000;
//		}
//
//		//		wheel_r.speed_r = common.old_ratio * wheel_r.old_speed_r + (1 - common.old_ratio) * wheel_r.speed_r;
//		//		wheel_l.speed_l = common.old_ratio * wheel_l.old_speed_l + (1 - common.old_ratio) * wheel_l.speed_l;
//
//		wheel_r.motor_r.SetPower(abs(wheel_r.speed_r));
//		wheel_l.motor_l.SetPower(abs(wheel_l.speed_l));
//
//
//	}
//}
//
//void myListener(const Byte *bytes, const size_t size)
//{
//	switch (bytes[0])
//	{
//	case 'z':
//		encoder_Kp += 0.1;
//		break;
//	case 'x':
//		if (encoder_Kp >= 0.1)
//			encoder_Kp -= 0.1;
//		break;
//
//	case 'Z':
//		encoder_Kp += 1;
//		break;
//	case 'X':
//		if (encoder_Kp >= 1)
//			encoder_Kp -= 1;
//		break;
//
//	case 'v':
//		encoder_Ki += 0.01;
//		break;
//	case 'b':
//		if (encoder_Ki >= 0.01)
//			encoder_Ki -= 0.01;
//		break;
//	case 'V':
//		encoder_Ki += 0.5;
//		break;
//	case 'B':
//		if (encoder_Ki >= 0.5)
//			encoder_Ki -= 0.5;
//		break;
//	case 'n':
//		encoder_Kd += 0.001;
//		break;
//	case 'm':
//		if(encoder_Kd >= 0.001)
//			encoder_Kd -= 0.001;
//		break;
//	case 'N':
//		encoder_Kd += 0.05;
//		break;
//	case 'M':
//		if(encoder_Kd >= 0.05)
//			encoder_Kd -= 0.05;
//		break;
//
//
//	}
//}
//
//
//
//int main()
//{
//	VarManager pGrapher;
//
//	//intialize the system
//	System::Init();
//	Timer::TimerInt t = 0;
//	Timer::TimerInt pt = t;
//	pt = System::Time();
//
//	/*
//
//	//Initalize the BT module
//
//	//	FtdiFt232r::Config bt_config;
//	//	bt_config.id = 0;
//	//	bt_config.rx_irq_threshold = 2;
//	//	// Set the baud rate (data transmission rate) to 115200 (this value must
//	//	// match the one set in the module, i.e., 115200, so you should not change
//	//	// here, or you won't be able to receive/transmit anything correctly)
//	//	bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//	//	FtdiFt232r bt(bt_config);
//	//	// Call EnableRx() to enable the BT module to receive data
//	//	bt.EnableRx();
//
//	//	Adc::Config Config;
//	//	Config.adc = Adc::Name::kAdc1Ad5B;
//	//	Config.resolution = Adc::Config::Resolution::k16Bit;
//	//	Adc LCCD(Config);
//
//	//	TowerProMg995::Config servoconfig;
//	//	servoconfig.id = 0;
//	//	TowerProMg995 servo(servoconfig);
//
//	 	//	Mma8451q::Config accel_config;
//	//	//sensitivity of accelerometer
//	//	accel_config.id = 0;
//	//	accel_config.scl_pin = Pin::Name::kPtb0;
//	//	accel_config.sda_pin = Pin::Name::kPtb1;
//	//	Mma8451q myAccel(accel_config);
//
//	 */
//
//	Mpu6050::Config gyro_config;
//	//sensitivity of gyro
//	gyro_config.gyro_range = Mpu6050::Config::Range::kLarge;
//	//sensitivity of accelerometer
//	gyro_config.accel_range = Mpu6050::Config::Range::kLarge;
//	Mpu6050 mpu6050(gyro_config);
//
//	LinearCcd ccd(0);
//
//	St7735r::Config config;
//	config.is_revert = false;
//	St7735r lcd(config);
//
//	LcdConsole::Config yoyo;
//	yoyo.bg_color = 0;
//	yoyo.text_color = -1;
//	yoyo.lcd = &lcd;
//	LcdConsole console(yoyo);
//
//
//	Gpo::Config howard;
//	howard.pin = Pin::Name::kPtc9;
//	Gpo lincoln(howard);
//
//
//	Joystick::Config joycon;
//	joycon.id = 0;
//	joycon.is_active_low = true;
//	Joystick joy(joycon);
//
//	LcdTypewriter::Config typeconfig;
//	typeconfig.lcd = &lcd;
//	typeconfig.bg_color = 0;
//	typeconfig.text_color = -1;
//	typeconfig.is_text_wrap = true;
//	LcdTypewriter type(typeconfig);
//
//	AlternateMotor::Config l_motor;
//	l_motor.id = 0;
//	AlternateMotor motor_r(l_motor);
//
//	DirEncoder::Config enconfig;
//	enconfig.id = 0;
//	DirEncoder encoder_l(enconfig);
//
//	DirEncoder::Config r_encoder;
//	r_encoder.id = 1;
//	DirEncoder encoder_r(r_encoder);
//
//	AlternateMotor::Config r_motor;
//	r_motor.id = 1;
//	AlternateMotor motor_l(r_motor);
//
//
//	lcd.Clear(0);
//	System::DelayMs(25);
//	pt= System::Time();
//	while(1){
//
//		mpu6050.Update();
//		System::DelayMs(4);
//		gyro_accel.accel = mpu6050.GetAccel();
//		original_angle = gyro_accel.accel[0]*57.29578;
//
//		t = System::Time();
//		if(t-pt <0){
//			pt=0;
//		}
//		if((t-pt)>=2000)
//			break;
//	}
//
//
//	//********************************************************************************************************************
//	//graph testing variable
//
//	//	pGrapher.addWatchedVar(&ir_Kp, "float", sizeof(float), "1");
//	//	pGrapher.addWatchedVar(&ir_Kd, "float", sizeof(float), "2");
//	//	pGrapher.addWatchedVar(&il_Kp, "float", sizeof(float), "3");
//	//	pGrapher.addWatchedVar(&il_Kd, "float", sizeof(float), "4");
//	//	pGrapher.addWatchedVar(&ic_Kp, "1");
//	//	pGrapher.addWatchedVar(&gyro_angle, "2");
//	//	pGrapher.addWatchedVar(&accel_angle, "3");
//	//	pGrapher.addWatchedVar(&output_angle, "4");
//	//	pGrapher.addWatchedVar(&count_r, "1");
//	//	pGrapher.addWatchedVar(&count_l, "2");
//	//	pGrapher.addWatchedVar(&output_angle, "3");
//	//	pGrapher.addWatchedVar(&ideal_count, "4");
//	//	pGrapher.addWatchedVar(&ic_Kp, "5");
//	//	pGrapher.addWatchedVar(&ic_Kd, "6");
//	//	pGrapher.addWatchedVar(&encoder_Ki, "7");
//
//	//	pGrapher.addWatchedVar(&kalman_value[0], "6");
//	//	pGrapher.addWatchedVar(&kalman_value[1], "7");
//
//	pGrapher.Init(&myListener);
//
//	double Q = 0.001;
//	double value[2] = {0.001, 0.2600496668};
//	double P;
//	Kalman kalman(Q, value, original_angle, 1);
//
//
//	encoder_r.Update();               //to reset the count
//	encoder_l.Update();
//	while(1){
//		if(t !=System::Time()){
//			t = System::Time();
//
//			if((int32_t)(t-pt1) >= 11  && yo==0){
//				//				lincoln.Turn();
//				pt1 = System::Time();
//				yo = 1;
//				Balance_function(wheel_l, wheel_r, original_angle, last_ideal_count, gyro_accel);
//
//			}
//
//
//			if((int32_t)(t-pt1) >= 3 && yo ==1){
//				//				lincoln.Turn();
//				pt2=System::Time();
//				yo = 2;
//				Follow_Encoder (wheel_l, wheel_r, common);
//			}
//
//			/*second round to get angle and encoder
//			 *
//			 *
//			 *
//			 *
//			 */
//			if((int32_t)(t-pt2) >= 2  && yo == 2){
//				//				lincoln.Turn();
//				pt3 = System::Time();
//
//				yo = 3;
//				Balance_function(wheel_l, wheel_r, original_angle, last_ideal_count, gyro_accel);
//
//			}
//
//
//			// Second round to follow the encoder
//			if((int32_t)(t-pt3) >= 3 && yo == 3){
//				//				lincoln.Turn();
//				pt4 = System::Time();
//				yo = 4;
//				Follow_Encoder (wheel_l, wheel_r, common);
//			}
//
//
//
//			if((int32_t)(t-pt4) >= 2 && yo ==4){
//				pt5 = System::Time();
//				yo =0;
//				pGrapher.sendWatchData();
//
//			}
//		}
//	}
//
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////
///*
// * lincoln_main.h
// *
// *  Created on: 2 Mar, 2015
// *      Author: lincoln
// */
//
//#include <libbase/k60/mcg.h>
//#include <libsc/led.h>
//#include <libsc/system.h>
//#include <libsc/ftdi_ft232r.h>
//#include <libsc/alternate_motor.h>
//#include <libsc/tower_pro_mg995.h>
//#include <libsc/mpu6050.h>
//#include <libsc/encoder.h>
//#include <libsc/dir_motor.h>
//#include <libsc/mma8451q.h>
//#include <libsc/device_h/mma8451q.h>
//#include <cstdio>
//#include <math.h>
//#include <libsc/linear_ccd.h>
//#include "libsc/st7735r.h"
//#include <libsc/lcd_console.h>
//#include <libsc/lcd_typewriter.h>
//#include <libbase/k60/adc.h>
//#include <libsc/joystick.h>
//#include <libsc/dir_encoder.h>
//#include <libutil/string.h>
//#include <libutil/kalman_filter.h>
//#include <libsc/jy_mcu_bt_106.h>
//#include <libsc/ab_encoder.h>
//
//#include "../inc/MyVarManager.h"
//
//#define BLACK           0x0000
//#define BLUE            0x001F
//#define RED             0xF800
//#define GREEN           0x07E0
//#define CYAN            0x07FF
//#define MAGENTA         0xF81F
//#define YELLOW          0xFFE0
//#define WHITE           0xFFFF
//#define aabbss(v) ((v > 0)? v : -v)
//#define white_black     0
//#define black_white     1
//#define half_black       2
//#define half_white       3
//
//char CCD;
//using namespace libsc;
//
//namespace libbase
//{
//namespace k60
//{
//
//Mcg::Config Mcg::GetMcgConfig()
//{
//	Mcg::Config config;
//	//Do Not change
//	config.external_oscillator_khz = 50000;
//	//Set MU clock to 100MHz
//	config.core_clock_khz = 180000;
//	return config;
//}
//
//}
//}
//
///*
//void ReceiveListener(const Byte *bytes, const size_t size)
//{
//	if (size != 2)
//		return;
//	switch (byte[0])
//	{
//	case 1:
//		motor.SetPower(500);
//
//	}
//}
// */
//
//float ideal_count_Kd = 0;
//float ideal_count_Kp = 0;
//float error_kd = 0;
//float ic_Kp = 3.5;
//float ic_Kd = 0.035;
//float ic_Ki = 0;
//float gyro_Ki = 0;
//float ccd_Kp = 0;
//float ccd_Kd = 0;
//
//float encoder_r_Kp = 130;   // Recommend 28
//float encoder_r_Kd = 0.05;
//float encoder_r_Ki = 0;
//float encoder_l_Kp = 128;    // Recommend 115
//float encoder_l_Kd = 0.050;    // Recommend 0.05
//float encoder_l_Ki = 0;    // No need
//float original_angle = 0;
//float new_original_angle = 0;
//float turn[2] = { 1, 1 };
//float still_Ki = 0.000;
//float ratio_old = 0;
//float ratio_new = 1-ratio_old;
//int32_t first_count = 0;
//float trust_accel = 0.01;
//float trust_old_accel = 0;
//float trust_new_accel = 1- trust_old_accel;
//Byte l_edge = 0;
//Byte r_edge = 0;
//Byte l_color_flag = 0;
//Byte r_color_flag = 0;
//
//
//int32_t count_l =0;
//int32_t count_r =0;
//
//float lincoln1 = 0;
//
//int32_t ideal_count = 0;
//void myListener(const Byte *bytes, const size_t size)
//{
//	switch (bytes[0])
//	{
//	//	case 'j':
//	//		original_angle += 1;
//	//		break;
//	//	case 'k':
//	//		original_angle -= 1;
//	//		break;
//	//	case 'o':
//	//		original_angle = new_original_angle;
//	//		break;
//	//	case 'p':
//	//		new_original_angle = original_angle;
//	//		break;
//	case '8':
//		turn[0] = 0;
//		turn[1] = 1.3;
//		break;
//	case '9':
//		turn[0] = 1;
//		turn[1] = 1;
//		break;
//	case '0':
//		turn[0] = 1.3;
//		turn[1] = 0;
//		break;
//	case 'z':
//		if(ic_Kp > 0.1){
//			ic_Kp -= 0.1;
//		}
//		break;
//	case 'x':
//		ic_Kp += 0.1;
//		break;
//	case 'Z':
//		if(ic_Kp > 1){
//			ic_Kp -= 1;
//		}
//		break;
//	case 'X':
//		ic_Kp += 1;
//		break;
//	case 'c':
//		if(ic_Kd > 0.05){
//			ic_Kd -= 0.05;
//		}
//		break;
//	case 'v':
//		ic_Kd += 0.05;
//		break;
//	case 'C':
//		if(ic_Kd > 0.1){
//			ic_Kd -= 0.1;
//		}
//		break;
//	case 'V':
//		ic_Kd += 0.1;
//		break;
//	case 'n':
//		if(ic_Ki > 0.01){
//			ic_Ki -= 0.01;
//		}
//		break;
//	case 'm':
//		ic_Ki += 0.01;
//		break;
//	case 'N':
//		if(ic_Ki > 0.1){
//			ic_Ki -= 0.1;
//		}
//		break;
//	case 'M':
//		ic_Ki += 0.1;
//		break;
//	case 'h':
//		if(encoder_l_Kp > 0.1){
//			encoder_l_Kp -= 0.1;
//		}
//		break;
//	case 'j':
//		encoder_l_Kp += 0.1;
//		break;
//	case 'H':
//		if(encoder_l_Kp > 1){
//			encoder_l_Kp -= 1;
//		}
//		break;
//	case 'J':
//		encoder_l_Kp += 1;
//		break;
//	case 'k':
//		if(encoder_l_Kd > 0.01){
//			encoder_l_Kd -= 0.01;
//		}
//		break;
//	case 'l':
//		encoder_l_Kd += 0.01;
//		break;
//	case 'K':
//		if(encoder_l_Kd > 0.1){
//			encoder_l_Kd -= 0.1;
//		}
//		break;
//	case 'L':
//		encoder_l_Kd += 0.1;
//		break;
//	case 'u':
//		if(encoder_r_Kp > 0.1){
//			encoder_r_Kp -= 0.1;
//		}
//		break;
//	case 'i':
//		encoder_r_Kp += 0.1;
//		break;
//	case 'U':
//		if(encoder_r_Kp > 1){
//			encoder_r_Kp -= 1;
//		}
//		break;
//	case 'I':
//		encoder_r_Kp += 1;
//		break;
//	case 'o':
//		if(encoder_r_Kd > 0.01){
//			encoder_r_Kd -= 0.01;
//		}
//		break;
//	case 'p':
//		encoder_r_Kd += 0.01;
//		break;
//	case 'O':
//		if(encoder_r_Kd > 0.1){
//			encoder_r_Kd -= 0.1;
//		}
//		break;
//	case 'P':
//		encoder_r_Kd += 0.1;
//		break;
//
//
//
//	}
//}
//
//
//int main()
//{
//	std::array<float, 3>accel;
//	std::array<float, 3>angle;
//	std::array<float, 3>omega;
//
//
//	uint16_t result;
//	char *buffer = new char[125]{0};
//	char *words = new char[125]{0};
//	uint16_t pixel1[2100];
//	uint16_t pixel_bg_colour[2100];
//	uint32_t avg = 0;
//	uint32_t all = 0;
//	std::array<uint16_t,LinearCcd::kSensorW> pixel;
//	uint16_t now_5pixel_value = 0;
//	uint16_t last_5pixel_value = 0;
//	float pixel_difference_sum = 0;
//	float pixel_avg_difference = 0;
//	float window = 5.0;
//	float window_avg = 0;
//	int state = 0;
//
//	Byte i=0;
//	const char *screen1 = "Interstellar\n\n>Sensor State\n\n Balance Mode\n\n Come Back Mode\n\n Run Forest!!!";
//	const char *screen2 ="Interstellar\n"
//			"\n"
//			" Sensor State\n"
//			"\n"
//			">Balance Mode\n"
//			"\n"
//			" Come Back Mode\n"
//			"\n"
//			" Run Forest!!!";
//	char screen3[] = "Interstellar\n"
//			"\n"
//			" Sensor State\n"
//			"\n"
//			" Balance Mode\n"
//			"\n"
//			">Come Back Mode\n"
//			"\n"
//			" Run Forest!!!";
//	char screen4[] = "Interstellar\n"
//			"\n"
//			" Sensor State\n"
//			"\n"
//			" Balance Mode\n"
//			"\n"
//			" Come Back Mode\n"
//			"\n"
//			">Run Forest!!!";
//
//	char screen5[] = "Go motor!!!";
//
//
//
//	VarManager pGrapher;
//
//
//
//	//intialize the system
//	System::Init();
//	Timer::TimerInt t = 0;
//	Timer::TimerInt pt = t;
//	pt = System::Time();
//
//	//	//	Initalize the BT module
//	//	JyMcuBt106::Config bt_config;
//	//	bt_config.id = 0;
//	//	bt_config.rx_irq_threshold = 2;
//	//	// Set the baud rate (data transmission rate) to 115200 (this value must
//	//	// match the one set in the module, i.e., 115200, so you should not change
//	//	// here, or you won't be able to receive/transmit anything correctly)
//	//	bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//	//	JyMcuBt106 bt(bt_config);
//
//	//	FtdiFt232r::Config uart_config;
//	//	uart_config.id = 0;
//	//	uart_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//	//	FtdiFt232r fu(uart_config);
//
//	//	Adc::Config Config;
//	//	Config.adc = Adc::Name::kAdc1Ad5B;
//	//	Config.resolution = Adc::Config::Resolution::k16Bit;
//	//	Adc LCCD(Config);
//
//	//	TowerProMg995::Config servoconfig;
//	//	servoconfig.id = 0;
//	//	TowerProMg995 servo(servoconfig);
//
//
//	Mpu6050::Config gyro_config;
//	//sensitivity of gyro
//	gyro_config.gyro_range = Mpu6050::Config::Range::kLarge;
//	//sensitivity of accelerometer
//	gyro_config.accel_range = Mpu6050::Config::Range::kLarge;
//	Mpu6050 mpu6050(gyro_config);
//
//
//	//	Mma8451q::Config accel_config;
//	//	//sensitivity of accelerometer
//	//	accel_config.id = 0;
//	//	accel_config.scl_pin = Pin::Name::kPtb0;
//	//	accel_config.sda_pin = Pin::Name::kPtb1;
//	//	Mma8451q myAccel(accel_config);
//
//	Joystick::Config joycon;
//	joycon.id = 0;
//	joycon.is_active_low = true;
//	Joystick joy(joycon);
//
//	AlternateMotor::Config l_motor;
//	l_motor.id = 1;
//	AlternateMotor motor_r(l_motor);
//
//	AlternateMotor::Config r_motor;
//	r_motor.id = 0;
//	AlternateMotor motor_l(r_motor);
//
//	AbEncoder::Config enconfig;
//	enconfig.id = 0;
//	AbEncoder encoder_l(enconfig);
//
//	AbEncoder::Config r_encoder;
//	r_encoder.id = 1;
//	AbEncoder encoder_r(r_encoder);
//
//	System::DelayMs(25);
//
//	int dead_value_l = 200;
//	int dead_value_r = 200;
//
//	while(1){
//		motor_l.SetClockwise(0);
//		motor_r.SetClockwise(0);
//		motor_l.SetPower(dead_value_l);
//		motor_r.SetPower(dead_value_r);
//		encoder_l.Update();
//		encoder_r.Update();
//		System::DelayMs(10);
//		encoder_l.Update();
//		encoder_r.Update();
//		if(encoder_l.GetCount())
//			dead_value_l -= 3;
//		if(encoder_r.GetCount())
//			dead_value_r -= 3;
//		if(encoder_l.GetCount() == 0 && encoder_r.GetCount() ==0)
//		{
//			while(1){
//				motor_l.SetPower(dead_value_l);
//				motor_r.SetPower(dead_value_r);
//				encoder_l.Update();
//				encoder_r.Update();
//				System::DelayMs(10);
//				encoder_l.Update();
//				encoder_r.Update();
//				count_l = encoder_l.GetCount();
//				count_r = -encoder_r.GetCount();
//				if(count_l <= 0)
//					dead_value_l += 3;
//				if(count_r <= 0)
//					dead_value_r += 3;
//				if(count_l > 0 && count_l > 0)
//					goto finish;
//			}
//		}
//	}
//
//	finish:
//	//		motor_l.SetPower(0);
//	//		motor_r.SetPower(0);
//	count_l = 0;
//	count_r = 0;
//
//
//	float raw_angle;
//	t= System::Time();
//	pt = t;
//	while(1){
//
//		mpu6050.Update();
//		System::DelayMs(4);
//		accel = mpu6050.GetAccel();
//		raw_angle = accel[0]*57.29578;
//
//		t = System::Time();
//		if(t-pt <0)
//			pt=0;
//		if((t-pt)>=2000)
//			break;
//	}
//
//	original_angle = raw_angle;
//
//	float accel_angle = original_angle;
//	float last_gyro_angle = original_angle;
//	float gyro_angle = original_angle;
//	float last_accel_angle = original_angle;
//	float output_angle ;            //karmen filtered
//
//	float mg = 0;
//	float total_count_l =0;
//	float total_count_r =0;
//	int last_encoder_error = 0;
//	int now_encoder_error = 0;
//	float last_angle_error = 0;
//	float now_angle_error = 0;
//	float angle_error_change = 0;
//
//	int32_t last_il_encoder_error = 0;
//	int32_t last_ir_encoder_error = 0;
//	int32_t ir_encoder_error = 0;
//	int32_t il_encoder_error = 0;
//	int32_t ir_encoder_error_change = 0;
//	int32_t il_encoder_error_change = 0;
//	int32_t last_ir_encoder_error_change = 0;
//	int32_t last_il_encoder_error_change = 0;
//	int32_t ir_encoder_errorsum = 0;
//	int32_t il_encoder_errorsum = 0;
//
//
//	//	KF m_gyro_kf[3];
//	//	float kalman_value[2] = {0.3f, 1.5f};
//	//	kalman_filter_init(&m_gyro_kf[0], 0.01f, &kalman_value, original_angle, 1);
//	int32_t speed_l = 0;                    //last output to motor left,0-1000
//	int32_t speed_r = 0;                    //last output to motor right,0-1000
//
//
//	uint32_t pt1 = 0;
//	uint32_t pt2 = 0;
//	uint32_t pt3 = 0;
//	uint32_t pt4 = 0;
//	uint32_t pt5 = 0;
//
//
//	int square_sign = 0;      //for the disappearance of -sign in power two fucntion
//	int sign = 0;
//	int last_sign = 0;
//	int last_ideal_count = 0;
//	int now_ideal_count = 0;
//	int error_count = 0;            //*Kp
//	float error_count_change = 0;   //*Kd
//	int last_error_count = 0;
//	int now_error_count = 0;
//
//	Byte yo = 0;                      //to organize the sequence of code
//	LinearCcd ccd(0);
//	Byte center_line = 0;
//	Byte center_line_flag = 0;
//	int16_t center_line_error = 0;
//	float last_center_line_error = 0;
//	float now_center_line_error = 0;
//	float center_line_error_change = 0;
//	float road_length = 0;
//	//********************************************************************************************************************
//	//graph testing variable
//
//	//	pGrapher.addWatchedVar(&ir_Kp, "float", sizeof(float), "1");
//
//	//	pGrapher.Init(&myListener);
//
//
//	uint32_t pt_round1 = System::Time();
//	char received;
//
//	encoder_r.Update();               //to reset the count
//	encoder_l.Update();
//
//	encoder_r.Update();               //to reset the count
//	encoder_l.Update();
//	while(1){
//
//		if(t !=System::Time()){
//			t = System::Time();
//
//			//			int sine_time = (int) t;
//			//			float sine_count = 800 * sin(sine_time / 1000.0f);
//			//			ideal_count = (int32_t) sine_count;
//
//			if(total_count_r >= 10000)
//				total_count_r = 10000;
//			if(total_count_r <= -10000)
//				total_count_r = -10000;
//			if(total_count_l >= 10000)
//				total_count_l = 10000;
//			if(total_count_l <= -10000)
//				total_count_l = -10000;
//
//			//			if(t - pt_round1 >= 500)
//			//			{
//			//				original_angle = raw_angle;
//			//				turn[0] = 1.0f;
//			//				turn[1] = 1.0f;
//			//			}
//			if(t - pt1 <0 ||t - pt2 < 0 ||t- pt3 < 0){
//				pt1 = 0;
//				pt2 = 0;
//				pt3 = 0;
//			}
//
//
//			if(t%50==0){
//				ccd.StartSample();
//				while (!ccd.SampleProcess())
//				{}
//				pixel = ccd.GetData();
//
//				now_5pixel_value = pixel[57] + pixel[58] + pixel[59] + pixel[60] + pixel[61];
//				for(int i=62; i < 118; i = i+5){
//					last_5pixel_value = now_5pixel_value;
//					now_5pixel_value = pixel[i] + pixel[i+1] + pixel[i+2] + pixel[i+3] + pixel[i+4];
//					pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5;
//
//					if(i == 62){
//						pixel_avg_difference = pixel_difference_sum/((i-57)/5);
//						if(pixel_difference_sum >= 1500){
//							pixel_avg_difference = 1;
//						}
//
//					}
//
//					if(pixel_avg_difference >= 10){
//						pixel_avg_difference = 1;
//					}
//					if((now_5pixel_value - last_5pixel_value)/5 < -150*aabbss(pixel_avg_difference)){
//						r_edge = i;
//						r_color_flag = white_black;
//						pixel_difference_sum = 0;
//						break;
//					}
//
//
//					else if((now_5pixel_value - last_5pixel_value)/5 > 150*aabbss(pixel_avg_difference)){
//						r_edge = i;
//						r_color_flag = black_white;
//						pixel_difference_sum = 0;
//						break;
//					}
//					else if(i == 117){
//						if((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 > 60000){
//							r_color_flag = half_white;
//							pixel_difference_sum = 0;
//							break;
//						}
//						else if((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 < 35000){
//							r_color_flag = half_black;
//							pixel_difference_sum = 0;
//							break;
//						}
//					}
//					pixel_avg_difference = pixel_difference_sum/((i-57)/5);
//				}
//
//				now_5pixel_value = pixel[65] + pixel[66] + pixel[67] + pixel[68] + pixel[69];
//				for(int i = 69; i > 8; i = i-5){
//					last_5pixel_value = now_5pixel_value;
//					now_5pixel_value = pixel[i] + pixel[i-1] + pixel[i-2] + pixel[i-3] + pixel[i-4];
//					pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5;
//
//					if(i == 69){
//						pixel_avg_difference = pixel_difference_sum/((74-i)/5);
//						if(pixel_difference_sum >= 1500){
//							pixel_avg_difference = 1;
//						}
//					}
//					if(pixel_avg_difference >= 10){
//						pixel_avg_difference = 1;
//					}
//
//					if((now_5pixel_value - last_5pixel_value)/5 < -150*aabbss(pixel_avg_difference)){
//						l_edge = i;
//						l_color_flag = white_black;
//						pixel_difference_sum = 0;
//						break;
//					}
//
//
//					else if((now_5pixel_value - last_5pixel_value)/5 > 150*aabbss(pixel_avg_difference)){
//						l_edge = i;
//						l_color_flag = black_white;
//						pixel_difference_sum = 0;
//						break;
//					}
//					else if(i == 9){
//
//						if((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 > 60000){
//							l_color_flag = half_white;
//							pixel_difference_sum = 0;
//							break;
//						}
//						else if((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 < 35000){
//							l_color_flag = half_black;
//							pixel_difference_sum = 0;
//							break;
//						}
//						break;
//					}
//					pixel_avg_difference = pixel_difference_sum/((74 - i)/5);
//				}
//
//				if(center_line_flag ==0){
//					road_length = r_edge - l_edge;
//					center_line = (road_length)/2+l_edge;
//					center_line_flag++;
//				}
//
//				last_center_line_error = now_center_line_error;
//				now_center_line_error = center_line - ((r_edge - l_edge)/2+l_edge);
//				center_line_error_change = now_center_line_error - last_center_line_error;
//				//
//				//				if(now_center_line_error >=0){
//				//					turn[0] = ccd_Kp*now_center_line_error + ccd_Kd*center_line_error_change;
//				//					turn[1] =
//				//				}
//				//
//
//			}
//			if((int32_t)(t-pt1) >= 11  && yo==0){
//				//				//				lincoln.Turn();
//				pt1 = System::Time();
//				//
//				yo = 1;
//
//				encoder_r.Update();
//				encoder_l.Update();
//
//				count_r = (int32_t)(-encoder_r.GetCount());
//				count_l = (int32_t)(encoder_l.GetCount());
//
//				total_count_l += (float)count_l * 0.002;
//				total_count_r += (float)count_r * 0.002;
//
//				last_ir_encoder_error = ir_encoder_error;
//				ir_encoder_error = ideal_count - count_r;
//				last_il_encoder_error = il_encoder_error;
//				il_encoder_error = ideal_count - count_l;
//
//				last_il_encoder_error_change = il_encoder_error -last_il_encoder_error;
//				last_ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;
//
//				mpu6050.Update();
//
//				accel = mpu6050.GetAccel();
//				omega = mpu6050.GetOmega();
//
//				last_accel_angle = accel_angle;
//				accel_angle = accel[0]*57.29578;
//				accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;
//
//				last_gyro_angle = gyro_angle;
//				gyro_angle += (-1) * omega[1] * 0.005 + trust_accel * (accel_angle - gyro_angle);
//
//				//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
//				//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
//				//				output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;
//
//				output_angle = gyro_angle;
//				last_angle_error = now_angle_error;
//				now_angle_error = original_angle - output_angle;
//				angle_error_change = now_angle_error -last_angle_error;
//
//				//				int angle_gain = 1;
//				//				if(now_angle_error > 4 || now_angle_error < -4)
//				//					angle_gain = 3;
//				//				else if(now_angle_error > 4 || now_angle_error < -4)
//				//					angle_gain = 1000;
//				//				else if(now_angle_error > 8 || now_angle_error < -8)
//				//					angle_gain = 2000;
//				//				else if(now_angle_error > 14 || now_angle_error < -14)
//				//					angle_gain = 3000;
//
//				last_ideal_count = ideal_count;
//				ideal_count = (int32_t)(ic_Kp * now_angle_error + ic_Kd * angle_error_change / 0.003 - still_Ki * total_count_r);
//				ideal_count = (int32_t)(0.2*last_ideal_count + 0.8*ideal_count);
//				//				angle_gain = 1;
//
//
//			}
//
//
//
//			if((int32_t)(t-pt1) >= 3 && yo ==1){
//				//				lincoln.Turn();
//				pt2=System::Time();
//
//				//				pt1 = t;
//				yo = 2;
//
//				encoder_r.Update();
//				encoder_l.Update();
//
//				count_r = (int32_t)(-encoder_r.GetCount());
//				count_l = (int32_t)(encoder_l.GetCount());
//
//				total_count_l += (float)count_l * 0.003;
//				total_count_r += (float)count_r * 0.003;
//
//				last_sign = sign;
//				if(ideal_count > 0){
//					sign = 0;
//				}
//				else if(ideal_count < 0){
//					sign = 1;
//				}
//				else if(ideal_count ==0){
//					sign = 2;
//				}
//
//
//				if(sign == 2){
//					motor_l.SetPower(0);
//					motor_r.SetPower(0);
//				}
//
//				else{
//
//					if(last_sign != sign){
//						motor_l.SetPower(0);
//						motor_r.SetPower(0);
//						motor_l.SetClockwise(sign);
//						motor_r.SetClockwise(sign);
//					}
//
//					last_ir_encoder_error = ir_encoder_error;
//					ir_encoder_error = ideal_count - count_r;
//					last_il_encoder_error = il_encoder_error;
//					il_encoder_error = ideal_count - count_l;
//
//
//					il_encoder_error_change = il_encoder_error -last_il_encoder_error;
//					ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;
//
//					ir_encoder_error_change = 0.8f * ir_encoder_error_change + 0.2f * last_ir_encoder_error_change;
//					il_encoder_error_change = 0.8f * il_encoder_error_change + 0.2f * last_il_encoder_error_change;
//
//					//					ir_encoder_errorsum -= ir_encoder_error;
//					//					il_encoder_errorsum -= il_encoder_error;
//
//
//					//					float hehe = 1;
//					//					if(ideal_count > 400 || ideal_count < -400)
//					//						hehe = 16.0f;
//					//					else if (ideal_count < 40 || ideal_count > -40)
//					//						hehe = 0.2f;
//					//					else if(ideal_count > 600 || ideal_count < -600)
//					//						hehe = 100.0f;
//
//
//
//										speed_r = (int32_t)(((ir_encoder_error+2) *encoder_r_Kp) + ir_encoder_error_change * encoder_r_Kd/0.003f + ir_encoder_errorsum*encoder_r_Ki*0.003f);
//										speed_l = (int32_t)(((il_encoder_error+2) *encoder_l_Kp ) + il_encoder_error_change * encoder_l_Kd/0.003f + il_encoder_errorsum*encoder_l_Ki*0.003f);
////					speed_r = (int32_t)(mg*now_angle_error);
////					speed_l = (int32_t)(mg*now_angle_error);
//
//
//
//
//					//					hehe = 1;
//
//
//					if(speed_l > 900 && sign ==0){
//						speed_l = 900;
//					}
//					else if(speed_l < 0 && sign ==0){
//						speed_l = 0;
//					}
//					else if(speed_l > 0 && sign ==1){
//						speed_l = 0;
//					}
//					else if(speed_l < -900 && sign ==1){
//						speed_l = -900;
//					}
//
//
//					if(speed_r > 900 && sign ==0){
//						speed_r = 900;
//					}
//					else if(speed_r < 0 && sign ==0){
//						speed_r = 0;
//					}
//					else if(speed_r > 0 && sign ==1){
//						speed_r = 0;
//					}
//					else if(speed_r < -900 && sign ==1){
//						speed_r = -900;
//					}
//
//					speed_r = speed_r*turn[1];
//					speed_l = speed_l*turn[0];
//
//
//					motor_l.SetPower(abs(speed_l)+dead_value_l);
//					motor_r.SetPower(abs(speed_r)+dead_value_r);
//
//
//				}
//
//
//			}
//
//
//			/*second round to get angle and encoder
//			 *
//			 *
//			 *
//			 *
//			 */
//			if((int32_t)(t-pt2) >= 2  && yo == 2){
//				//				lincoln.Turn();
//				pt3 = System::Time();
//
//				yo = 3;
//
//				encoder_r.Update();
//				encoder_l.Update();
//
//				count_r = (int32_t)(-encoder_r.GetCount());
//				count_l = (int32_t)(encoder_l.GetCount());
//
//				last_ir_encoder_error = ir_encoder_error;
//				ir_encoder_error = ideal_count - count_r;
//				last_il_encoder_error = il_encoder_error;
//				il_encoder_error = ideal_count - count_l;
//
//				last_il_encoder_error_change = il_encoder_error -last_il_encoder_error;
//				last_ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;
//
//				total_count_l += (float)count_l * 0.002;
//				total_count_r += (float)count_r * 0.002;
//
//				mpu6050.Update();
//
//				accel = mpu6050.GetAccel();
//				omega = mpu6050.GetOmega();
//
//				last_accel_angle = accel_angle;
//				accel_angle = accel[0]*57.29578;
//				accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;
//
//				last_gyro_angle = gyro_angle;
//				gyro_angle += (-1) *omega[1]*0.005 + trust_accel * (accel_angle - gyro_angle);
//				//			gyro_angle = accel_angle+(-1) *omega[0];
//				//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;
//
//
//
//				//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
//				//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
//				//				output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;
//
//				output_angle = gyro_angle;
//				last_angle_error = now_angle_error;
//				now_angle_error = original_angle - output_angle;
//				angle_error_change = now_angle_error -last_angle_error;
//
//				//				int angle_gain = 1;
//				//				if(now_angle_error > 4 || now_angle_error < -4)
//				//					angle_gain = 3;
//				//				else if(now_angle_error > 6 || now_angle_error < -6)
//				//					angle_gain = 150;
//				//				else if(now_angle_error > 8 || now_angle_error < -8)
//				//					angle_gain = 1200;
//				//				else if(now_angle_error > 14 || now_angle_error < -14)
//				//					angle_gain = 3000;
//
//				last_ideal_count = ideal_count;
//				ideal_count = (int32_t)(ic_Kp * now_angle_error+ ic_Kd * angle_error_change / 0.003 - still_Ki * total_count_r);
//				ideal_count = (int32_t)(0.2 * last_ideal_count + 0.8 * ideal_count);
//				//				angle_gain = 1;
//
//			}
//
//
//			if((int32_t)(t-pt3) >= 3 && yo == 3){
//				//				lincoln.Turn();
//				pt4 = System::Time();
//
//				//				pt1 = t;
//				yo = 4;
//
//
//				encoder_r.Update();
//				encoder_l.Update();
//
//				count_r = (int32_t)(-encoder_r.GetCount());
//				count_l = (int32_t)(encoder_l.GetCount());
//
//				total_count_l += (float)count_l * 0.003;
//				total_count_r += (float)count_r * 0.003;
//
//				last_sign = sign;
//				if(ideal_count > 0){
//					sign = 0;
//				}
//				else if(ideal_count < 0){
//					sign = 1;
//				}
//				else if(ideal_count ==0){
//					sign = 2;
//				}
//
//
//				if(sign == 2){
//					motor_l.SetPower(0);
//					motor_r.SetPower(0);
//				}
//
//				else{
//
//					if(last_sign != sign){
//						motor_l.SetPower(0);
//						motor_r.SetPower(0);
//						motor_l.SetClockwise(sign);
//						motor_r.SetClockwise(sign);
//					}
//
//					last_ir_encoder_error = ir_encoder_error;
//					ir_encoder_error = ideal_count - count_r;
//					last_il_encoder_error = il_encoder_error;
//					il_encoder_error = ideal_count - count_l;
//
//
//					il_encoder_error_change = il_encoder_error -last_il_encoder_error;
//					ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;
//
//					ir_encoder_error_change = 0.8 * ir_encoder_error_change + 0.2 * last_ir_encoder_error_change;
//					il_encoder_error_change = 0.8 * il_encoder_error_change + 0.2 * last_il_encoder_error_change;
//
//					ir_encoder_errorsum -= ir_encoder_error;
//					il_encoder_errorsum -= il_encoder_error;
//
//
//					//					lincoln1 = (float)(ir_encoder_error_change * encoder_r_Kd/3);
//
//					//					float hehe = 1;
//					//					if(ideal_count > 400 || ideal_count < -400)
//					//						hehe = 16.0f;
//					//					else if (ideal_count < 60 || ideal_count > -60)
//					//						hehe = 0.2f;
//					//					else if(ideal_count > 600 || ideal_count < -600)
//					//						hehe = 100.0f;
//
//
//
//
//					//					speed_r = (int32_t)((float)(mg*(original_angle-output_angle)+(ir_encoder_error+2) *encoder_r_Kp) + ir_encoder_error_change * encoder_r_Kd/0.003f + ir_encoder_errorsum*encoder_r_Ki*0.003f);
//					//					speed_l = (int32_t)((float)(mg*(original_angle-output_angle)+(il_encoder_error+2) *encoder_l_Kp ) + il_encoder_error_change * encoder_l_Kd/0.003f + il_encoder_errorsum*encoder_l_Ki*0.003f);
//					speed_r = (int32_t)(mg*now_angle_error);
//					speed_l = (int32_t)(mg*now_angle_error);
//
//
//					//					hehe = 1;
//
//					if(speed_l > 900 && sign == 0){
//						speed_l = 900;
//					}
//					else if(speed_l < 0 && sign ==0){
//						speed_l = 0;
//					}
//					else if(speed_l > 0 && sign ==1){
//						speed_l = 0;
//					}
//					else if(speed_l < -900 && sign ==1){
//						speed_l = -900;
//					}
//
//
//					if(speed_r > 900 && sign ==0){
//						speed_r = 900;
//					}
//					else if(speed_r < 0 && sign ==0){
//						speed_r = 0;
//					}
//					else if(speed_r > 0 && sign ==1){
//						speed_r = 0;
//					}
//					else if(speed_r < -900 && sign ==1){
//						speed_r = -900;
//					}
//
//					speed_r = speed_r*turn[1];
//					speed_l = speed_l*turn[0];
//
//
//					motor_l.SetPower(abs(speed_l)+dead_value_l);
//					motor_r.SetPower(abs(speed_r)+dead_value_r);
//
//
//				}
//
//
//			}
//
//
//			if((int32_t)(t-pt4) >= 2 && yo ==4){
//				pt5 = System::Time();
//				yo =0;
//				encoder_r.Update();
//				encoder_l.Update();
//
//				count_r = (int32_t)(-encoder_r.GetCount());
//				count_l = (int32_t)(encoder_l.GetCount());
//
//				total_count_l += (float)count_l * 0.001;
//				total_count_r += (float)count_r * 0.001;
//
//				pGrapher.sendWatchData();
//				//				bt.SendStr(libutil::String::Format(" %f, %d, %f, %f, %f, %d\n",output_angle, ideal_count,
//				//						ic_Kp, ic_Kd, still_Ki, count_l ).c_str());
//
//			}
//
//
//		}
//
//	}
//
//
//}
//
//
////	motor_l.SetPower(200);
////	motor_r.SetPower(200);
////	motor_l.SetClockwise(0);
////	motor_r.SetClockwise(0);
////
////
////	while(1){
////
////		encoder_l.Update();
////		encoder_r.Update();
////		System::DelayMs(50);
////		int count_l =  - encoder_l.GetCount();
////		int count_r = encoder_r.GetCount();
////
////
////	}
//
//
//
//
//
//
