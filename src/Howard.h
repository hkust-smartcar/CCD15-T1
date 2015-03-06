/*
 * main.cpp
 *
 *  Created on: 26 Dec, 2014
 *      Author: Howard
 */

#include <libbase/k60/mcg.h>
#include <libsc/k60/led.h>
#include <libsc/k60/system.h>
#include <libsc/k60/ftdi_ft232r.h>
#include <libsc/k60/alternate_motor.h>
#include <libsc/k60/tower_pro_mg995.h>
#include <libsc/k60/mpu6050.h>
#include <libsc/k60/encoder.h>
#include <libsc/k60/dir_motor.h>
#include <libsc/k60/mma8451q.h>
#include <libsc/device_h/mma8451q.h>
#include <cstdio>
#include <libsc/k60/linear_ccd.h>
#include "libsc/k60/st7735r.h"
#include <libsc/k60/lcd_console.h>
#include <libsc/k60/lcd_typewriter.h>
#include <libbase/k60/adc.h>
#include <libsc/k60/joystick.h>
#include <libsc/k60/dir_encoder.h>
#include <libutil/string.h>
#include <libutil/kalman_filter.h>
#include "VarManager.h"
#include <kalman.h>

#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

float ideal_count_Kd = 0;
float ideal_count_Kp = 0;
float error_kd = 0;
float ic_Kp = 7.5;
float ic_Ki = 0;
float ic_Kd = 0.68;
float gyro_Ki = 0;
float encoder_Kp = 35;
float encoder_Ki = 0;
float encoder_Kd = 0.03;

int32_t last_ideal_count = 0;
int32_t ideal_count = 0;
float original_angle = 0;
Byte i=0;

uint32_t pt1 = 0;
uint32_t pt2 = 0;
uint32_t pt3 = 0;
uint32_t pt4 = 0;
uint32_t pt5 = 0;

Byte yo = 0;                      //to organize the sequence of code



class Wheel_l{
	DirEncoder encoder_l;
	int32_t count_l = 0;
	AlternateMotor motor_l;
	int32_t last_il_encoder_error = 0;
	int32_t il_encoder_error = 0;
	int32_t il_encoder_error_change = 0;
	int32_t il_encoder_errorsum = 0;
	int32_t speed_l = 0;
	int32_t old_speed_l = 0;
} wheel_l;


class Wheel_r{
public:
	DirEncoder encoder_r;
	int32_t count_r = 0;
	AlternateMotor motor_r;
	int32_t last_ir_encoder_error = 0;
	int32_t ir_encoder_error = 0;
	int32_t ir_encoder_error_change = 0;
	int32_t ir_encoder_errorsum = 0;
	int32_t speed_r = 0;
	int32_t old_speed_r = 0;

} wheel_r;

class Common_Para{
public:
	int last_sign = 0;
	int sign = 0;
	float old_ratio = 0.75;
}  common;

class Gyro_Accel{
public:
	Mpu6050 mpu6050;
	std::array<float, 3>accel;
	std::array<float, 3>angle;
	std::array<float, 3>omega;
	double accel_angle = original_angle;
	double last_gyro_angle = original_angle;
	double gyro_angle = original_angle;
	double last_accel_angle = original_angle;
	double output_angle;
	float trust_gyro = 1;
	float trust_accel = 1 - trust_gyro;
	double last_angle_error = 0;
	double now_angle_error = 0;
	double angle_error_change;
} gyro_accel;

class Lcd{
	uint16_t result;
	char *buffer = new char[125]{0};
	char *words = new char[125]{0};
	uint16_t pixel1[2100];
	uint16_t pixel_bg_colour[2100];
	uint32_t avg = 0;
	uint32_t all = 0;
	std::array<uint16_t,LinearCcd::kSensorW> pixel;
	float window = 5.0;
	float window_avg = 0;
	int state = 0;

	const char *screen1 = "Interstellar\n\n>Sensor State\n\n Balance Mode\n\n Come Back Mode\n\n Run Forest!!!";
	const char *screen2 ="Interstellar\n"
			"\n"
			" Sensor State\n"
			"\n"
			">Balance Mode\n"
			"\n"
			" Come Back Mode\n"
			"\n"
			" Run Forest!!!";
	char screen3[] = "Interstellar\n"
			"\n"
			" Sensor State\n"
			"\n"
			" Balance Mode\n"
			"\n"
			">Come Back Mode\n"
			"\n"
			" Run Forest!!!";
	char screen4[] = "Interstellar\n"
			"\n"
			" Sensor State\n"
			"\n"
			" Balance Mode\n"
			"\n"
			" Come Back Mode\n"
			"\n"
			">Run Forest!!!";

	char screen5[] = "Go motor!!!";
} lcd;



void Balance_function(Wheel_l &wheel_l, Wheel_r &wheel_r, float original_angle, int &last_ideal_count, Gyro_Accel &gyro_accel);
void Follow_Encoder (Wheel_l &wheel_l, Wheel_r &wheel_r, Common_Para &common);

char CCD;
using namespace libsc::k60;

namespace libbase
{
namespace k60
{

Mcg::Config Mcg::GetMcgConfig()
{
	Mcg::Config config;
	//Do Not change
	config.external_oscillator_khz = 50000;
	//Set MU clock to 100MHz
	config.core_clock_khz = 180000;
	return config;
}

}
}


void Follow_Encoder ();

void Balance_function(Wheel_l &wheel_l, Wheel_r &wheel_r, float original_angle, int &last_ideal_count, Gyro_Accel &gyro_accel, Kalman kalman)
{
	wheel_r.encoder_r.Update();
	wheel_l.encoder_l.Update();

	gyro_accel.mpu6050.Update();

	std::array<float, 3>accel;
	std::array<float, 3>omega;

	accel = gyro_accel.mpu6050.GetAccel();
	omega = gyro_accel.mpu6050.GetOmega();

	gyro_accel.last_accel_angle = gyro_accel.accel_angle;
	gyro_accel.accel_angle = accel[0]*57.29578;
	gyro_accel.accel_angle = 0.65*gyro_accel.last_accel_angle +0.35*gyro_accel.accel_angle;

	gyro_accel.last_gyro_angle = gyro_accel.gyro_angle;
	gyro_accel.gyro_angle += (-1) *omega[1]*0.005 + 0.01*(gyro_accel.accel_angle - gyro_accel.gyro_angle);
	//			gyro_angle = accel_angle+(-1) *omega[0];
	//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;


	kalman.Filtering(&gyro_accel.output_angle, gyro_accel.gyro_angle, gyro_accel.accel_angle);

	//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
	//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
	//	gyro_accel.output_angle = gyro_accel.trust_gyro*gyro_accel.gyro_angle +gyro_accel.trust_accel * gyro_accel.accel_angle;




	gyro_accel.last_angle_error = gyro_accel.now_angle_error;
	gyro_accel.now_angle_error = gyro_accel.output_angle - original_angle;
	gyro_accel.angle_error_change = gyro_accel.now_angle_error - gyro_accel.last_angle_error;

	last_ideal_count = ideal_count;
	int temp_sign = 1;
	if (gyro_accel.now_angle_error < 0)
		temp_sign = -1;
	ideal_count = (int32_t)(ic_Kp * temp_sign * gyro_accel.now_angle_error + ic_Kd * gyro_accel.angle_error_change);

	ideal_count = (int32_t)(0.5 * last_ideal_count + 0.5 * ideal_count);
}





void Follow_Encoder (Wheel_l &wheel_l, Wheel_r &wheel_r, Common_Para &common)
{
	wheel_r.encoder_r.Update();
	wheel_l.encoder_l.Update();

	wheel_r.count_r = -1 * wheel_r.encoder_r.GetCount() / 3;
	wheel_l.count_l = wheel_l.encoder_l.GetCount() / 3;

	common.last_sign = common.sign;
	if(ideal_count > 0){
		common.sign = 1;
	}
	else if(ideal_count < 0){
		common.sign = 0;
	}
	else if(ideal_count ==0){
		common.sign = 2;
	}


	if(common.sign == 2){
		wheel_l.motor_l.SetPower(0);
		wheel_r.motor_r.SetPower(0);
	}

	else{

		if(common.last_sign != common.sign){
			wheel_l.motor_l.SetPower(0);
			wheel_r.motor_r.SetPower(0);
			wheel_l.motor_l.SetClockwise(common.sign);
			wheel_r.motor_r.SetClockwise(common.sign);
		}

		wheel_r.last_ir_encoder_error = wheel_r.ir_encoder_error;
		wheel_r.ir_encoder_error = ideal_count - wheel_r.count_r;
		wheel_l.last_il_encoder_error = wheel_l.il_encoder_error;
		wheel_l.il_encoder_error = ideal_count - wheel_l.count_l;


		wheel_l.il_encoder_error_change = wheel_l.il_encoder_error -wheel_l.last_il_encoder_error;
		wheel_r.ir_encoder_error_change = wheel_r.ir_encoder_error -wheel_r.last_ir_encoder_error;

		wheel_r.old_speed_r = wheel_r.speed_r;
		wheel_l.old_speed_l = wheel_l.speed_l;

		wheel_r.ir_encoder_errorsum += wheel_r.ir_encoder_error * 0.003;
		wheel_l.il_encoder_errorsum += wheel_l.il_encoder_error * 0.003;

		wheel_r.speed_r = (int32_t)(wheel_r.ir_encoder_error * encoder_Kp
				+ wheel_r.ir_encoder_error_change*encoder_Kd/0.003
				+ wheel_r.ir_encoder_errorsum * encoder_Ki);
		wheel_l.speed_l = (int32_t)(wheel_l.il_encoder_error * encoder_Kp
				+ wheel_l.il_encoder_error_change*encoder_Kd/0.003
				+ wheel_l.il_encoder_errorsum * encoder_Ki);
		if(wheel_l.speed_l > 1000 && common.sign ==1){
			wheel_l.speed_l = 1000;
		}
		else if(wheel_l.speed_l < 0 && common.sign ==1){
			wheel_l.speed_l = 0;
		}
		else if(wheel_l.speed_l > 0 && common.sign ==0){
			wheel_l.speed_l = 0;
		}
		else if(wheel_l.speed_l < -1000 && common.sign ==0){
			wheel_l.speed_l = -1000;
		}


		if(wheel_r.speed_r > 1000 && common.sign ==1){
			wheel_r.speed_r = 1000;
		}
		else if(wheel_r.speed_r < 0 && common.sign ==1){
			wheel_r.speed_r = 0;
		}
		else if(wheel_r.speed_r > 0 && common.sign ==0){
			wheel_r.speed_r = 0;
		}
		else if(wheel_r.speed_r < -1000 && common.sign ==0){
			wheel_r.speed_r = -1000;
		}

		//		wheel_r.speed_r = common.old_ratio * wheel_r.old_speed_r + (1 - common.old_ratio) * wheel_r.speed_r;
		//		wheel_l.speed_l = common.old_ratio * wheel_l.old_speed_l + (1 - common.old_ratio) * wheel_l.speed_l;

		wheel_r.motor_r.SetPower(abs(wheel_r.speed_r));
		wheel_l.motor_l.SetPower(abs(wheel_l.speed_l));


	}
}

void myListener(const Byte *bytes, const size_t size)
{
	switch (bytes[0])
	{
	case 'z':
		encoder_Kp += 0.1;
		break;
	case 'x':
		if (encoder_Kp >= 0.1)
			encoder_Kp -= 0.1;
		break;

	case 'Z':
		encoder_Kp += 1;
		break;
	case 'X':
		if (encoder_Kp >= 1)
			encoder_Kp -= 1;
		break;

	case 'v':
		encoder_Ki += 0.01;
		break;
	case 'b':
		if (encoder_Ki >= 0.01)
			encoder_Ki -= 0.01;
		break;
	case 'V':
		encoder_Ki += 0.5;
		break;
	case 'B':
		if (encoder_Ki >= 0.5)
			encoder_Ki -= 0.5;
		break;
	case 'n':
		encoder_Kd += 0.001;
		break;
	case 'm':
		if(encoder_Kd >= 0.001)
			encoder_Kd -= 0.001;
		break;
	case 'N':
		encoder_Kd += 0.05;
		break;
	case 'M':
		if(encoder_Kd >= 0.05)
			encoder_Kd -= 0.05;
		break;


	}
}



int main()
{
	VarManager pGrapher;

	//intialize the system
	System::Init();
	Timer::TimerInt t = 0;
	Timer::TimerInt pt = t;
	pt = System::Time();

	/*

	//Initalize the BT module

	//	FtdiFt232r::Config bt_config;
	//	bt_config.id = 0;
	//	bt_config.rx_irq_threshold = 2;
	//	// Set the baud rate (data transmission rate) to 115200 (this value must
	//	// match the one set in the module, i.e., 115200, so you should not change
	//	// here, or you won't be able to receive/transmit anything correctly)
	//	bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	//	FtdiFt232r bt(bt_config);
	//	// Call EnableRx() to enable the BT module to receive data
	//	bt.EnableRx();

	//	Adc::Config Config;
	//	Config.adc = Adc::Name::kAdc1Ad5B;
	//	Config.resolution = Adc::Config::Resolution::k16Bit;
	//	Adc LCCD(Config);

	//	TowerProMg995::Config servoconfig;
	//	servoconfig.id = 0;
	//	TowerProMg995 servo(servoconfig);

	 	//	Mma8451q::Config accel_config;
	//	//sensitivity of accelerometer
	//	accel_config.id = 0;
	//	accel_config.scl_pin = Pin::Name::kPtb0;
	//	accel_config.sda_pin = Pin::Name::kPtb1;
	//	Mma8451q myAccel(accel_config);

	 */

	Mpu6050::Config gyro_config;
	//sensitivity of gyro
	gyro_config.gyro_range = Mpu6050::Config::Range::kLarge;
	//sensitivity of accelerometer
	gyro_config.accel_range = Mpu6050::Config::Range::kLarge;
	Mpu6050 mpu6050(gyro_config);

	LinearCcd ccd(0);

	St7735r::Config config;
	config.is_revert = false;
	St7735r lcd(config);

	LcdConsole::Config yoyo;
	yoyo.bg_color = 0;
	yoyo.text_color = -1;
	yoyo.lcd = &lcd;
	LcdConsole console(yoyo);


	Gpo::Config howard;
	howard.pin = Pin::Name::kPtc9;
	Gpo lincoln(howard);


	Joystick::Config joycon;
	joycon.id = 0;
	joycon.is_active_low = true;
	Joystick joy(joycon);

	LcdTypewriter::Config typeconfig;
	typeconfig.lcd = &lcd;
	typeconfig.bg_color = 0;
	typeconfig.text_color = -1;
	typeconfig.is_text_wrap = true;
	LcdTypewriter type(typeconfig);

	AlternateMotor::Config l_motor;
	l_motor.id = 0;
	AlternateMotor motor_r(l_motor);

	DirEncoder::Config enconfig;
	enconfig.id = 0;
	DirEncoder encoder_l(enconfig);

	DirEncoder::Config r_encoder;
	r_encoder.id = 1;
	DirEncoder encoder_r(r_encoder);

	AlternateMotor::Config r_motor;
	r_motor.id = 1;
	AlternateMotor motor_l(r_motor);


	lcd.Clear(0);
	System::DelayMs(25);
	pt= System::Time();
	while(1){

		mpu6050.Update();
		System::DelayMs(4);
		gyro_accel.accel = mpu6050.GetAccel();
		original_angle = gyro_accel.accel[0]*57.29578;

		t = System::Time();
		if(t-pt <0){
			pt=0;
		}
		if((t-pt)>=2000)
			break;
	}


	//********************************************************************************************************************
	//graph testing variable

	//	pGrapher.addWatchedVar(&ir_Kp, "float", sizeof(float), "1");
	//	pGrapher.addWatchedVar(&ir_Kd, "float", sizeof(float), "2");
	//	pGrapher.addWatchedVar(&il_Kp, "float", sizeof(float), "3");
	//	pGrapher.addWatchedVar(&il_Kd, "float", sizeof(float), "4");
	//	pGrapher.addWatchedVar(&ic_Kp, "1");
	//	pGrapher.addWatchedVar(&gyro_angle, "2");
	//	pGrapher.addWatchedVar(&accel_angle, "3");
	//	pGrapher.addWatchedVar(&output_angle, "4");
	//	pGrapher.addWatchedVar(&count_r, "1");
	//	pGrapher.addWatchedVar(&count_l, "2");
	//	pGrapher.addWatchedVar(&output_angle, "3");
	//	pGrapher.addWatchedVar(&ideal_count, "4");
	//	pGrapher.addWatchedVar(&ic_Kp, "5");
	//	pGrapher.addWatchedVar(&ic_Kd, "6");
	//	pGrapher.addWatchedVar(&encoder_Ki, "7");

	//	pGrapher.addWatchedVar(&kalman_value[0], "6");
	//	pGrapher.addWatchedVar(&kalman_value[1], "7");

	pGrapher.Init(&myListener);

	double Q = 0.001;
	double value[2] = {0.001, 0.2600496668};
	double P;
	Kalman kalman(Q, value, original_angle, 1);


	encoder_r.Update();               //to reset the count
	encoder_l.Update();
	while(1){
		if(t !=System::Time()){
			t = System::Time();

			if((int32_t)(t-pt1) >= 11  && yo==0){
				//				lincoln.Turn();
				pt1 = System::Time();
				yo = 1;
				Balance_function(wheel_l, wheel_r, original_angle, last_ideal_count, gyro_accel);

			}


			if((int32_t)(t-pt1) >= 3 && yo ==1){
				//				lincoln.Turn();
				pt2=System::Time();
				yo = 2;
				Follow_Encoder (wheel_l, wheel_r, common);
			}

			/*second round to get angle and encoder
			 *
			 *
			 *
			 *
			 */
			if((int32_t)(t-pt2) >= 2  && yo == 2){
				//				lincoln.Turn();
				pt3 = System::Time();

				yo = 3;
				Balance_function(wheel_l, wheel_r, original_angle, last_ideal_count, gyro_accel);

			}


			// Second round to follow the encoder
			if((int32_t)(t-pt3) >= 3 && yo == 3){
				//				lincoln.Turn();
				pt4 = System::Time();
				yo = 4;
				Follow_Encoder (wheel_l, wheel_r, common);
			}



			if((int32_t)(t-pt4) >= 2 && yo ==4){
				pt5 = System::Time();
				yo =0;
				pGrapher.sendWatchData();

			}
		}
	}

}








