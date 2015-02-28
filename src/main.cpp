/*
 * main.cpp
 *
 *  Created on: 26 Dec, 2014
 *      Author: lincoln
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

#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

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

void Balance_function(DirEncoder encoder_r, DirEncoder encoder_l, Mpu6050 mpu6050, int &last_sign,
		float &last_accel_angle, float &accel_angle, float &last_gyro_angle, float &gyro_angle,
		float &output_angle, float trust_gyro, float trust_accel,float &last_angle_error,
		float &now_angle_error, float original_angle, float &angle_error_change, int &last_ideal_count);

void Follow_Encoder (DirEncoder encoder_r, DirEncoder encoder_l, int32_t &count_l, int32_t &count_r,
		int &last_sign, int &sign, AlternateMotor motor_r, AlternateMotor motor_l,
		int32_t &last_ir_encoder_error, int32_t &last_il_encoder_error,
		int32_t &ir_encoder_error, int32_t &il_encoder_error,
		int32_t &ir_encoder_error_change, int32_t &il_encoder_error_change,
		int32_t &ir_encoder_errorsum, 	int32_t &il_encoder_errorsum,
		int32_t &speed_l, int32_t &speed_r, int32_t &old_speed_l, int32_t &old_speed_r);

/*
void ReceiveListener(const Byte *bytes, const size_t size)
{
	if (size != 2)
		return;
	switch (byte[0])
	{
	case 1:
		motor.SetPower(500);

	}
}
 */
// void Stop()

float ideal_count_Kd = 0;
float ideal_count_Kp = 0;
float error_kd = 0;
float ic_Kd = 0.68;
float ic_Kp = 7.5;
float gyro_Ki = 0;
float encoder_Kp = 35;
float encoder_Kd = 0.03;
float encoder_Ki = 0;

//float howard =0;

int32_t ideal_count = 0;
void myListener(const Byte *bytes, const size_t size)
{
	switch (bytes[0])
	{
	case 'z':
		ic_Kp += 0.5;
		break;
	case 'x':
		if (ic_Kp >= 0.5)
			ic_Kp -= 0.5;
		break;
	case 'v':
		encoder_Ki += 0.01;
		break;
	case 'b':
		if (encoder_Ki >= 0.01)
			encoder_Ki -= 0.01;
		break;
	case 'n':
		encoder_Kp += 0.5;
		break;
	case 'm':
		if(encoder_Kp >= 0.5)
			encoder_Kp -= 0.5;
		break;
	case 'j':
		encoder_Kd += 0.01;
		break;
	case 'k':
		if(encoder_Kd >= 0.01)
			encoder_Kd -= 0.01;
		break;


	}
}


int main()
{
	std::array<float, 3>accel;
	std::array<float, 3>angle;
	std::array<float, 3>omega;


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

	float original_angle = -12.4667;

	Byte i=0;
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



	VarManager pGrapher;



	//intialize the system
	System::Init();
	Timer::TimerInt t = 0;
	Timer::TimerInt pt = t;
	pt = System::Time();


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



	//	FtdiFt232r::Config uart_config;
	//	uart_config.id = 0;
	//	uart_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	//	FtdiFt232r fu(uart_config);
	// Initialize other things as necessary...


	//
	//	Adc::Config Config;
	//	Config.adc = Adc::Name::kAdc1Ad5B;
	//	Config.resolution = Adc::Config::Resolution::k16Bit;
	//	Adc LCCD(Config);

	//	AlternateMotor::Config config;
	//	config.id = 0;
	//	AlternateMotor motor(config);

	//	TowerProMg995::Config servoconfig;
	//	servoconfig.id = 0;
	//	TowerProMg995 servo(servoconfig);

	Mpu6050::Config gyro_config;
	//sensitivity of gyro
	gyro_config.gyro_range = Mpu6050::Config::Range::kLarge;
	//sensitivity of accelerometer
	gyro_config.accel_range = Mpu6050::Config::Range::kLarge;
	Mpu6050 mpu6050(gyro_config);


	//	Mma8451q::Config accel_config;
	//	//sensitivity of accelerometer
	//	accel_config.id = 0;
	//	accel_config.scl_pin = Pin::Name::kPtb0;
	//	accel_config.sda_pin = Pin::Name::kPtb1;
	//	Mma8451q myAccel(accel_config);


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

	AlternateMotor::Config l_motor;
	l_motor.id = 0;
	AlternateMotor motor_r(l_motor);

	AlternateMotor::Config r_motor;
	r_motor.id = 1;
	AlternateMotor motor_l(r_motor);

	DirEncoder::Config enconfig;
	enconfig.id = 0;
	DirEncoder encoder_l(enconfig);

	DirEncoder::Config r_encoder;
	r_encoder.id = 1;
	DirEncoder encoder_r(r_encoder);

	LcdTypewriter::Config typeconfig;
	typeconfig.lcd = &lcd;
	typeconfig.bg_color = 0;
	typeconfig.text_color = -1;
	typeconfig.is_text_wrap = true;
	LcdTypewriter type(typeconfig);



	lcd.Clear(0);
	System::DelayMs(25);

	float trust_gyro = 1;
	float trust_accel = 1-trust_gyro;
//	pt= System::Time();
	//	while(1){
	//
	//		mpu6050.Update();
	//		System::DelayMs(4);
	//		accel = mpu6050.GetAccel();
	//		original_angle = accel[0]*57.29578;
	//
	//		//		console.SetCursorRow(4);
	//		//		sprintf(buffer, "angle:%.3f",original_angle);
	//		//		console.WriteString((char*)buffer);
	//		//		System::DelayMs(150);
	//		t = System::Time();
	//		if(t-pt <0){
	//			pt=0;
	//		}
	//		if((t-pt)>=2000){
	//
	//			break;
	//
	//		}
	//	}

	float accel_angle = original_angle;
	float last_gyro_angle = original_angle;
	float gyro_angle = original_angle;
	float last_accel_angle = original_angle;
	float output_angle ;            //karmen filtered
	int32_t count_l =0;
	int32_t count_r =0;
	int last_encoder_error = 0;
	int now_encoder_error = 0;
	float last_angle_error = 0;
	float now_angle_error = 0;
	float angle_error_change = 0;

	int32_t last_il_encoder_error = 0;
	int32_t last_ir_encoder_error = 0;
	int32_t ir_encoder_error = 0;
	int32_t il_encoder_error = 0;
	int32_t ir_encoder_error_change = 0;
	int32_t il_encoder_error_change = 0;
	int32_t ir_encoder_errorsum = 0;
	int32_t il_encoder_errorsum = 0;


	//	KF m_gyro_kf[3];
	//	float kalman_value[2] = {0.3f, 1.5f};
	//	kalman_filter_init(&m_gyro_kf[0], 0.01f, &kalman_value, original_angle, 1);
	int32_t speed_l = 0;                    //last output to motor left,0-1000
	int32_t speed_r = 0;                    //last output to motor right,0-1000


	uint32_t pt1 = 0;
	uint32_t pt2 = 0;
	uint32_t pt3 = 0;
	uint32_t pt4 = 0;
	uint32_t pt5 = 0;


	int sign = 0;            //for the disappearance of -sign in power two function
	int last_sign = 0;
	int last_ideal_count = 0;
	int now_ideal_count = 0;
	int error_count = 0;            //*Kp
	float error_count_change = 0;   //*Kd
	int last_error_count = 0;
	int now_error_count = 0;


	int32_t old_speed_l = 0;
	int32_t old_speed_r = 0;


	float doitonce = 0;
	Byte getout = 0;

	Byte yo = 0;                      //to organize the sequence of code
	int32_t total_error_ir = 0;       //Pi
	int32_t total_error_il = 0;       //Pi
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
	pGrapher.addWatchedVar(&count_r, "1");
	pGrapher.addWatchedVar(&count_l, "2");
	pGrapher.addWatchedVar(&output_angle, "3");
	pGrapher.addWatchedVar(&ideal_count, "4");
	pGrapher.addWatchedVar(&ic_Kp, "5");
	pGrapher.addWatchedVar(&ic_Kd, "6");
	pGrapher.addWatchedVar(&encoder_Ki, "7");

	//	pGrapher.addWatchedVar(&kalman_value[0], "6");
	//	pGrapher.addWatchedVar(&kalman_value[1], "7");

	pGrapher.Init(&myListener);



	encoder_r.Update();               //to reset the count
	encoder_l.Update();
	while(1){
		if(t !=System::Time()){
			t = System::Time();

			if((int32_t)(t-pt1) >= 11  && yo==0){
				//				lincoln.Turn();
				pt1 = System::Time();

				yo = 1;

				Balance_function(encoder_r, encoder_l, mpu6050, last_sign,
						last_accel_angle, accel_angle, last_gyro_angle, gyro_angle,
						output_angle, trust_gyro, trust_accel,last_angle_error,
						now_angle_error, original_angle, angle_error_change, last_ideal_count);

			}


			if((int32_t)(t-pt1) >= 3 && yo ==1){
				//				lincoln.Turn();
				pt2=System::Time();

				//				pt1 = t;
				yo = 2;

				Follow_Encoder (encoder_r, encoder_l, count_l, count_r,
						last_sign, sign, motor_r,  motor_l,
						last_ir_encoder_error, last_il_encoder_error,
						ir_encoder_error, il_encoder_error,
						ir_encoder_error_change, il_encoder_error_change,
						ir_encoder_errorsum, il_encoder_errorsum,
						speed_l, speed_r, old_speed_l, old_speed_r);

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

				Balance_function(encoder_r, encoder_l, mpu6050, last_sign,
						last_accel_angle, accel_angle, last_gyro_angle, gyro_angle,
						output_angle, trust_gyro, trust_accel,last_angle_error,
						now_angle_error, original_angle, angle_error_change, last_ideal_count);

			}



			// Second round to follow the encoder
			if((int32_t)(t-pt3) >= 3 && yo == 3){
				//				lincoln.Turn();
				pt4 = System::Time();

				//				pt1 = t;
				yo = 4;

				Follow_Encoder (encoder_r, encoder_l, count_l, count_r,
						last_sign, sign, motor_r,  motor_l,
						last_ir_encoder_error, last_il_encoder_error,
						ir_encoder_error, il_encoder_error,
						ir_encoder_error_change, il_encoder_error_change,
						ir_encoder_errorsum, il_encoder_errorsum,
						speed_l, speed_r, old_speed_l, old_speed_r);



			}



			if((int32_t)(t-pt4) >= 2 && yo ==4){
				pt5 = System::Time();
				yo =0;
				pGrapher.sendWatchData();

			}
		}
	}



}


void Balance_function(DirEncoder encoder_r, DirEncoder encoder_l, Mpu6050 mpu6050, int &last_sign,
		float &last_accel_angle, float &accel_angle, float &last_gyro_angle, float &gyro_angle,
		float &output_angle, float trust_gyro, float trust_accel,float &last_angle_error,
		float &now_angle_error, float original_angle, float &angle_error_change, int &last_ideal_count)
{
	encoder_r.Update();
	encoder_l.Update();


	//				yo++;
	mpu6050.Update();

	std::array<float, 3>accel;
	std::array<float, 3>omega;

	accel = mpu6050.GetAccel();
	omega = mpu6050.GetOmega();

	last_accel_angle = accel_angle;
	accel_angle = accel[0]*57.29578;
	accel_angle = 0.65*last_accel_angle +0.35*accel_angle;

	last_gyro_angle = gyro_angle;
	gyro_angle += (-1) *omega[1]*0.005+0.006706 + 0.01*(accel_angle - gyro_angle);
	//			gyro_angle = accel_angle+(-1) *omega[0];
	//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;



	//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
	//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
	output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;
	last_angle_error = now_angle_error;
	now_angle_error = output_angle - original_angle;
	angle_error_change = now_angle_error -last_angle_error;

	last_ideal_count = ideal_count;
	int temp_sign = 1;
	if (now_angle_error < 0)
		temp_sign = -1;
	ideal_count = (int32_t)(ic_Kp * temp_sign * now_angle_error * now_angle_error + ic_Kd * angle_error_change);

	ideal_count = (int32_t)(0.5*last_ideal_count + 0.5*ideal_count);
}







void Follow_Encoder (DirEncoder encoder_r, DirEncoder encoder_l, int32_t &count_l, int32_t &count_r,
		int &last_sign, int &sign, AlternateMotor motor_r, AlternateMotor motor_l,
		int32_t &last_ir_encoder_error, int32_t &last_il_encoder_error,
		int32_t &ir_encoder_error, int32_t &il_encoder_error,
		int32_t &ir_encoder_error_change, int32_t &il_encoder_error_change,
		int32_t &ir_encoder_errorsum, 	int32_t &il_encoder_errorsum,
		int32_t &speed_l, int32_t &speed_r, int32_t &old_speed_l, int32_t &old_speed_r)
{
	encoder_r.Update();
	encoder_l.Update();

	count_r = -1*encoder_r.GetCount() / 3 * 4;
	count_l = encoder_l.GetCount() / 3 * 4;

	last_sign = sign;
	if(ideal_count > 0){
		sign = 1;
	}
	else if(ideal_count < 0){
		sign = 0;
	}
	else if(ideal_count ==0){
		sign = 2;
	}


	if(sign == 2){
		motor_l.SetPower(0);
		motor_r.SetPower(0);
	}

	else{

		if(last_sign != sign){
			motor_l.SetPower(0);
			motor_r.SetPower(0);
			motor_l.SetClockwise(sign);
			motor_r.SetClockwise(sign);
		}

		last_ir_encoder_error = ir_encoder_error;
		ir_encoder_error = ideal_count - count_r;
		last_il_encoder_error = il_encoder_error;
		il_encoder_error = ideal_count - count_l;


		il_encoder_error_change = il_encoder_error -last_il_encoder_error;
		ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;

		old_speed_r = speed_r;
		old_speed_l = speed_l;

		ir_encoder_errorsum += count_r;
		il_encoder_errorsum += count_l;

		speed_r = (int32_t)(ir_encoder_error * encoder_Kp + ir_encoder_error_change*encoder_Kd/0.003 + ir_encoder_errorsum*encoder_Ki);
		speed_l = (int32_t)(il_encoder_error * encoder_Kp + il_encoder_error_change*encoder_Kd/0.003 + il_encoder_errorsum*encoder_Ki);
		if(speed_l > 1000 && sign ==1){
			speed_l = 1000;
		}
		else if(speed_l < 0 && sign ==1){
			speed_l = 0;
		}
		else if(speed_l > 0 && sign ==0){
			speed_l = 0;
		}
		else if(speed_l < -1000 && sign ==0){
			speed_l = -1000;
		}


		if(speed_r > 1000 && sign ==1){
			speed_r = 1000;
		}
		else if(speed_r < 0 && sign ==1){
			speed_r = 0;
		}
		else if(speed_r > 0 && sign ==0){
			speed_r = 0;
		}
		else if(speed_r < -1000 && sign ==0){
			speed_r = -1000;
		}

		speed_r = 0.75*old_speed_r + 0.25*speed_r;
		speed_l = 0.75*old_speed_l + 0.25*speed_l;

		motor_r.SetPower(abs(speed_r));
		motor_l.SetPower(abs(speed_l));


	}
}


