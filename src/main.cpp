/*
 * lincoln_main.h
 *
 *  Created on: 2 Mar, 2015
 *      Author: lincoln
 */

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
float ic_Kp = 3.9;
float gyro_Ki = 0;
float encoder_Kp = 100;
float encoder_Kd = 0;
float encoder_Ki = 0;
float original_angle = -12.24;
float new_original_angle = 0;
float turn[2] = { 1, 1 };
float still_Kp = 0;
float still_Kd = 0;
float ratio_old = 0.27;
float ratio_new = 1-ratio_old;

//float howard =0;
float lincoln1 = 0;

int32_t ideal_count = 0;
void myListener(const Byte *bytes, const size_t size)
{
	switch (bytes[0])
	{
	//	case 'j':
	//		original_angle += 1;
	//		break;
	//	case 'k':
	//		original_angle -= 1;
	//		break;
	//	case 'o':
	//		original_angle = new_original_angle;
	//		break;
	//	case 'p':
	//		new_original_angle = original_angle;
	//		break;
	case '8':
		turn[0] = 0;
		turn[1] = 1.2;
		break;
	case '9':
		turn[0] = 1;
		turn[1] = 1;
		break;
	case '0':
		turn[0] = 1.2;
		turn[1] = 0;
		break;
	case 'z':
		encoder_Kp += 1;
		break;
	case 'Z':

		encoder_Kp += 20;

		break;
	case 'x':
		if(encoder_Kp >=1){
			encoder_Kp -= 1;
		}
		break;
	case 'X':
		if(encoder_Kp >=20){
			encoder_Kp -= 20;
		}
		break;
	case 'c':
		encoder_Kd +=1;
		break;
	case 'C':
		encoder_Kd += 20;
		break;
	case 'v':
		if(encoder_Kd >=1){
			encoder_Kd -= 1;
		}
		break;
	case 'V':
		if(encoder_Kd >= 20){
			encoder_Kd -= 20;
		}
		break;
	case 'n':

		encoder_Ki += 0.05;
		break;
	case 'm':
		if(encoder_Ki >=0.05)
			encoder_Ki -= 0.05;
		break;
	case 'N':

		encoder_Ki += 20;
		break;
	case 'M':
		if(encoder_Ki >= 20)
			encoder_Ki -= 20;
		break;
	case 'j':
		ideal_count += 10;
		break;
	case 'k':
		ideal_count -= 10;
		break;
		//	case 'N':
		//		if(ratio_old <= 1 && ratio_old >=0)
		//			ratio_old += 0.1;
		//		break;
		//	case 'M':
		//		if(ratio_old >=0.1)
		//			ratio_old -= 0.1;
		//		break;
		//	case 'a':
		//		ideal_count += 5;
		//		break;
		//	case 's':
		//		ideal_count -= 5;
		//		break;
		//	case 'A':
		//		ideal_count += 80;
		//		break;
		//	case 'S':
		//		ideal_count -= 80;
		//		break;
		//
	case 'g':
		ic_Kp -= 0.1;
		break;
	case 'h':
		ic_Kp += 0.1;
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
	float raw_angle;
	pt= System::Time();
	while(1){

		mpu6050.Update();
		System::DelayMs(4);
		accel = mpu6050.GetAccel();
		raw_angle = accel[0]*57.29578;

		//		console.SetCursorRow(4);
		//		sprintf(buffer, "angle:%.3f",original_angle);
		//		console.WriteString((char*)buffer);
		//		System::DelayMs(150);
		t = System::Time();
		if(t-pt <0){
			pt=0;
		}
		if((t-pt)>=2000){

			break;

		}
	}
	original_angle = raw_angle;

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


	int square_sign = 0;      //for the disappearance of -sign in power two fucntion
	int sign = 0;
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
	pGrapher.addWatchedVar(&ideal_count, "1");
	pGrapher.addWatchedVar(&count_r, "2");
	pGrapher.addWatchedVar(&count_l, "3");
	pGrapher.addWatchedVar(&encoder_Kp, "4");
	pGrapher.addWatchedVar(&ir_encoder_error, "5");
	pGrapher.addWatchedVar(&encoder_Kd, "6");
	pGrapher.addWatchedVar(&encoder_Ki, "7");
	pGrapher.addWatchedVar(&il_encoder_errorsum, "8");
	//	pGrapher.addWatchedVar(&ir_encoder_errorsum, "9");


	//	pGrapher.addWatchedVar(&kalman_value[0], "6");
	//	pGrapher.addWatchedVar(&kalman_value[1], "7");

	pGrapher.Init(&myListener);



	encoder_r.Update();               //to reset the count
	encoder_l.Update();
	while(1){





		if(t !=System::Time()){
			t = System::Time();
			//		if(t - pt1 <0 ||t - pt2 < 0 ||t- pt3 < 0){
			//			pt1 = 0;
			//			pt2 = 0;
			//			pt3 = 0;
			//		}

			//		 && (t- pt2) >=2


			//		speed_r = encoder_value*10;
			//		motor_r.SetPower(speed_r);
			//		motor_l.SetPower(speed_r);


			if((int32_t)(t-pt1) >= 11  && yo==0){
				//
				//
				//				//				lincoln.Turn();
				pt1 = System::Time();
				//						motor_l.SetPower(0);
				motor_r.SetPower(0);
				yo = 1;

				//
				//
				//				//				yo++;
				//				mpu6050.Update();
				//
				//				accel = mpu6050.GetAccel();
				//				omega = mpu6050.GetOmega();
				//
				//				last_accel_angle = accel_angle;
				//				accel_angle = accel[0]*57.29578;
				//				accel_angle = 0.65*last_accel_angle +0.35*accel_angle;
				//
				//				last_gyro_angle = gyro_angle;
				//				gyro_angle += (-1) *omega[1]*0.005+0.006706 + 0.0098*(accel_angle - gyro_angle);
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
				//				//				if(output_angle - raw_angle <5 && output_angle - raw_angle > -5){
				//				//					original_angle = 0.8*original_angle +0.2*raw_angle;
				//				//				}
				//				//				else if(output_angle - original_angle >=5){
				//				//					original_angle -= (output_angle - raw_angle-5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
				//				//				}
				//				//				else if(output_angle - original_angle <=-5){
				//				//					original_angle -= (output_angle - raw_angle + 5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
				//				//				}
				//
				//
				//
				//
				//				now_angle_error = output_angle - original_angle;
				//				angle_error_change = now_angle_error -last_angle_error;
				//
				//				last_ideal_count = ideal_count;
				//				if(now_angle_error >= 0){
				//					square_sign = 1;
				//				}
				//				else if(now_angle_error < 0){
				//					square_sign = -1;
				//				}
				//				ideal_count = (int32_t)(ic_Kp*(now_angle_error)+ ic_Kd*angle_error_change);
				//				//			ideal_count = 3.1*(now_error)+ 0.9 * angle_error_change;
				//
				//				//				if(last_ideal_count >=0){
				//				//					last_sign = 1;
				//				//				}
				//				//				else{
				//				//					last_sign = 0;
				//				//				}
				//
				//				ideal_count = (int32_t)(0.5*last_ideal_count + 0.5*ideal_count);







			}






			//				lincoln.Set(0);
			//					if(output_angle -original_angle< 0){
			//						sign = -1;
			//					}
			//					else{
			//						sign = 1;
			//					}


			//			}








			//			if(t%15 ==0){
			//				lincoln.Turn();
			//
			//				ccd.StartSample();
			//				while (!ccd.SampleProcess())
			//				{}
			//				pixel = ccd.GetData();
			//
			//				for(int i = 0; i<LinearCcd::kSensorW; i++){
			//					all += pixel[i];
			//				}
			//				avg = (all / LinearCcd::kSensorW);
			//				all = 0;
			//
			//				for(int i=0;i<LinearCcd::kSensorW; i++){
			//					if(pixel[i] <avg){
			//						pixel[i] = 0;
			//					}
			//					else if(pixel[i]>=avg){
			//						pixel[i] = 1;
			//					}
			//				}
			//
			//				for(int i=0;i<LinearCcd::kSensorW-4; i++){
			//					window_avg = (pixel[i]+pixel[i+1]+pixel[i+2]+pixel[i+3]+pixel[i+4])/window;
			//					if(1-window_avg < window_avg){
			//						pixel[i+2] = 1;
			//					}
			//					else if(1-window_avg > window_avg){
			//						pixel[i+2] = 0;
			//					}
			//
			//				}
			//
			//
			//
			//
			//
			//			}







			//			//					error_count = ideal_count - count_r;
			//			//					last_error_count = now_error_count;
			//			//					now_error_count = error_count;
			//			//					error_count_change = now_error_count - last_error_count;
			//			//
			//
			//			//					while(now_error >30 ||now_error < -30){
			//			//						motor_l.SetPower(0);
			//			//						motor_r.SetPower(0);
			//			//					}
			//			//
			//			//					if(ideal_count > 1){
			//			//						motor_l.SetClockwise(1);
			//			//						motor_r.SetClockwise(1);
			//			//
			//			//						motor_speed += 1*(error_count) + 1*error_count_change;
			//			//						motor_l.SetPower(abs(motor_speed)+50);
			//			//						motor_r.SetPower(abs(motor_speed)+50);
			//			//					}
			//			//					else if(ideal_count <=1 &&ideal_count >=-1){
			//			//
			//			//						motor_l.SetPower(0);
			//			//						motor_r.SetPower(0);
			//			//					}
			//			//					else if(ideal_count < -1){
			//			//						motor_l.SetClockwise(0);
			//			//						motor_r.SetClockwise(0);
			//			//						motor_speed += 1*(error_count) + 0.1*error_count_change;
			//			//						motor_l.SetPower(abs(motor_speed)+50);
			//			//						motor_r.SetPower(abs(motor_speed)+50);
			//			//
			//			//					}
			//
			//
			//
			//





			if((int32_t)(t-pt1) >= 3 && yo ==1){
				//				lincoln.Turn();
				pt2=System::Time();

				//				pt1 = t;
				yo = 2;

				encoder_r.Update();
				encoder_l.Update();

				count_r = (int32_t)(-1 * encoder_r.GetCount() / 3);
				count_l = (int32_t)(encoder_l.GetCount() / 3);


				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count - count_r;
				last_il_encoder_error = il_encoder_error;
				il_encoder_error = ideal_count - count_l;


				il_encoder_error_change = il_encoder_error -last_il_encoder_error;
				ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;

				old_speed_r = speed_r;
				old_speed_l = speed_l;

				ir_encoder_errorsum += ir_encoder_error;
				il_encoder_errorsum += il_encoder_error;


				lincoln1 = (float)(ir_encoder_error_change * encoder_Kd / 3);

				speed_r = (int32_t)(ir_encoder_error * encoder_Kp + lincoln1 + ir_encoder_errorsum*encoder_Ki*0.003);
				speed_l = (int32_t)(il_encoder_error * encoder_Kp + (float)(il_encoder_error_change * encoder_Kd/3) + il_encoder_errorsum*encoder_Ki*0.003);

				speed_r = ratio_old * old_speed_r + ratio_new * speed_r;
				speed_l = ratio_old * old_speed_l + ratio_new * speed_l;

				speed_r = speed_r*turn[1];
				speed_l = speed_l*turn[0];

				if(speed_r >= 0)
					motor_r.SetPower(1);
				else
					motor_r.SetPower(0);

				if(speed_l >= 0)
					motor_l.SetPower(1);
				else
					motor_l.SetPower(0);

				motor_l.SetPower(abs(speed_l));
				motor_r.SetPower(abs(speed_r));


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

			encoder_r.Update();
			encoder_l.Update();
			//
			//
			//				//				yo++;
			//				mpu6050.Update();
			//
			//				accel = mpu6050.GetAccel();
			//				omega = mpu6050.GetOmega();
			//
			//				last_accel_angle = accel_angle;
			//				accel_angle = accel[0]*57.29578;
			//				accel_angle = 0.65*last_accel_angle +0.35*accel_angle;
			//
			//				last_gyro_angle = gyro_angle;
			//				gyro_angle += (-1) *omega[1]*0.005+0.006706 + 0.0098*(accel_angle - gyro_angle);
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
			//				//				if(output_angle - raw_angle <5 && output_angle - raw_angle > -5){
			//				//					original_angle = 0.8*original_angle +0.2*raw_angle;
			//				//				}
			//				//				else if(output_angle - original_angle >=5){
			//				//					original_angle -= (output_angle - raw_angle-5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
			//				//				}
			//				//				else if(output_angle - original_angle <=-5){
			//				//					original_angle -= (output_angle - raw_angle + 5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
			//				//				}
			//
			//
			//
			//
			//
			//
			//				now_angle_error = output_angle - original_angle;
			//				angle_error_change = now_angle_error -last_angle_error;
			//
			//				last_ideal_count = ideal_count;
			//				if(now_angle_error >= 0){
			//					square_sign = 1;
			//				}
			//				else if(now_angle_error < 0){
			//					square_sign = -1;
			//				}
			//				ideal_count = (int32_t)(ic_Kp*(now_angle_error)+ ic_Kd*angle_error_change);
			//				//			ideal_count = 3.1*(now_error)+ 0.9 * angle_error_change;
			//
			//				//				if(last_ideal_count >=0){
			//				//					last_sign = 1;
			//				//				}
			//				//				else{
			//				//					last_sign = 0;
			//				//				}
			//
			//				ideal_count = (int32_t)(0.5*last_ideal_count + 0.5*ideal_count);




		}





		if((int32_t)(t-pt3) >= 3 && yo == 3){
			//				lincoln.Turn();
			pt4 = System::Time();

			//				pt1 = t;
			yo = 4;


			encoder_r.Update();
			encoder_l.Update();

			count_r = (int32_t)(-1 * encoder_r.GetCount() / 3);
			count_l = (int32_t)(encoder_l.GetCount() / 3);


			last_ir_encoder_error = ir_encoder_error;
			ir_encoder_error = ideal_count - count_r;
			last_il_encoder_error = il_encoder_error;
			il_encoder_error = ideal_count - count_l;


			il_encoder_error_change = il_encoder_error -last_il_encoder_error;
			ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;

			old_speed_r = speed_r;
			old_speed_l = speed_l;

			ir_encoder_errorsum += ir_encoder_error;
			il_encoder_errorsum += il_encoder_error;


			lincoln1 = (float)(ir_encoder_error_change * encoder_Kd / 3);

			speed_r = (int32_t)(ir_encoder_error * encoder_Kp + lincoln1 + ir_encoder_errorsum*encoder_Ki*0.003);
			speed_l = (int32_t)(il_encoder_error * encoder_Kp + (float)(il_encoder_error_change * encoder_Kd/3) + il_encoder_errorsum*encoder_Ki*0.003);

			speed_r = ratio_old * old_speed_r + ratio_new * speed_r;
			speed_l = ratio_old * old_speed_l + ratio_new * speed_l;

			speed_r = speed_r*turn[1];
			speed_l = speed_l*turn[0];

			if(speed_r >= 0)
				motor_r.SetPower(1);
			else
				motor_r.SetPower(0);

			if(speed_l >= 0)
				motor_l.SetPower(1);
			else
				motor_l.SetPower(0);

			motor_l.SetPower(abs(speed_l));
			motor_r.SetPower(abs(speed_r));



		}







	if((int32_t)(t-pt4) >= 2 && yo ==4){
		pt5 = System::Time();
		yo =0;
		pGrapher.sendWatchData();

	}




}

}
}








