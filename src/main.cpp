/*
 * lincoln_main.h
 *
 *  Created on: 2 Mar, 2015
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
float ic_Kd = 0;
float ic_Kp = 0;
float ic_Ki = 0;
float gyro_Ki = 0;
//float encoder_Kp = 204;
//float encoder_Kd = 81;
float encoder_r_Kp = 0;
float encoder_r_Kd = 0;
float encoder_r_Ki = 0;
float encoder_l_Kp = 0;
float encoder_l_Kd = 0;
float encoder_l_Ki = 0;
float original_angle = -12.24;
float new_original_angle = 0;
float turn[2] = { 1, 1 };
float still_Kp = 0;
float still_Kd = 0;
float ratio_old = 0;
float ratio_new = 1-ratio_old;
int32_t first_count = 0;
float trust_accel = 0.016;
float trust_old_accel = 0.83;
float trust_new_accel = 1- trust_old_accel;


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
		turn[1] = 1.3;
		break;
	case '9':
		turn[0] = 1;
		turn[1] = 1;
		break;
	case '0':
		turn[0] = 1.3;
		turn[1] = 0;
		break;
	case 'z':
		if(ic_Kp > 0.1){
			ic_Kp -= 0.1;
		}
		break;
	case 'x':
		ic_Kp += 0.1;
		break;
	case 'Z':
		if(ic_Kp > 1){
			ic_Kp -= 1;
		}
		break;
	case 'X':
		ic_Kp += 1;
		break;
	case 'c':
		if(ic_Kd > 0.05){
			ic_Kd -= 0.05;
		}
		break;
	case 'v':
		ic_Kd += 0.05;
		break;
	case 'C':
		if(ic_Kd > 0.1){
			ic_Kd -= 0.1;
		}
		break;
	case 'V':
		ic_Kd += 0.1;
		break;
	case 'n':
		if(ic_Ki > 0.01){
			ic_Ki -= 0.01;
		}
		break;
	case 'm':
		ic_Ki += 0.01;
		break;
	case 'N':
		if(ic_Ki > 0.1){
			ic_Ki -= 0.1;
		}
		break;
	case 'M':
		ic_Ki += 0.1;
		break;
	case 'h':
		if(encoder_l_Kp > 0.1){
			encoder_l_Kp -= 0.1;
		}
		break;
	case 'j':
		encoder_l_Kp += 0.1;
		break;
	case 'H':
		if(encoder_l_Kp > 1){
			encoder_l_Kp -= 1;
		}
		break;
	case 'J':
		encoder_l_Kp += 1;
		break;
	case 'k':
		if(encoder_l_Kd > 0.01){
			encoder_l_Kd -= 0.01;
		}
		break;
	case 'l':
		encoder_l_Kd += 0.01;
		break;
	case 'K':
		if(encoder_l_Kd > 0.1){
			encoder_l_Kd -= 0.1;
		}
		break;
	case 'L':
		encoder_l_Kd += 0.1;
		break;
	case 'u':
		if(encoder_r_Kp > 0.1){
			encoder_r_Kp -= 0.1;
		}
		break;
	case 'i':
		encoder_r_Kp += 0.1;
		break;
	case 'U':
		if(encoder_r_Kp > 1){
			encoder_r_Kp -= 1;
		}
		break;
	case 'I':
		encoder_r_Kp += 1;
		break;
	case 'o':
		if(encoder_r_Kd > 0.01){
			encoder_r_Kd -= 0.01;
		}
		break;
	case 'p':
		encoder_r_Kd += 0.01;
		break;
	case 'O':
		if(encoder_r_Kd > 0.1){
			encoder_r_Kd -= 0.1;
		}
		break;
	case 'P':
		encoder_r_Kd += 0.1;
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
	l_motor.id = 1;
	AlternateMotor motor_r(l_motor);

	AlternateMotor::Config r_motor;
	r_motor.id = 0;
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
	//		pGrapher.addWatchedVar(&ir_Kd, "2");
	//		pGrapher.addWatchedVar(&il_Kp, "3");
	pGrapher.addWatchedVar(&ideal_count, "1");
	pGrapher.addWatchedVar(&ic_Kp, "2");
	pGrapher.addWatchedVar(&ic_Kd, "3");
	pGrapher.addWatchedVar(&encoder_r_Kp, "4");
	pGrapher.addWatchedVar(&encoder_r_Kd, "5");
	//	pGrapher.addWatchedVar(&ir_encoder_errorsum, "6");
	pGrapher.addWatchedVar(&output_angle, "6");
	pGrapher.addWatchedVar(&count_l, "7");
	pGrapher.addWatchedVar(&count_r, "8");

	//	pGrapher.addWatchedVar(&speed_l, "5");

	//	pGrapher.addWatchedVar(&speed_r, "7");
	//	pGrapher.addWatchedVar(&ratio_old, "8");



	//	pGrapher.addWatchedVar(&kalman_value[0], "6");
	//	pGrapher.addWatchedVar(&kalman_value[1], "7");

	pGrapher.Init(&myListener);



	encoder_r.Update();               //to reset the count
	encoder_l.Update();
	while(1){





		if(t !=System::Time()){
			t = System::Time();
			if(t - pt1 <0 ||t - pt2 < 0 ||t- pt3 < 0){
				pt1 = 0;
				pt2 = 0;
				pt3 = 0;
			}




			if((int32_t)(t-pt1) >= 11  && yo==0){
				//
				//
				//				//				lincoln.Turn();
				pt1 = System::Time();
				//
				yo = 1;

				encoder_r.Update();
				encoder_l.Update();

				mpu6050.Update();

				accel = mpu6050.GetAccel();
				omega = mpu6050.GetOmega();

				last_accel_angle = accel_angle;
				accel_angle = accel[0]*57.29578;
				accel_angle = trust_old_accel*last_accel_angle + trust_new_accel*accel_angle;

				last_gyro_angle = gyro_angle;
				gyro_angle += (-1) *omega[1]*0.005+0.006706 + trust_accel*(accel_angle - gyro_angle);
				//			gyro_angle = accel_angle+(-1) *omega[0];
				//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;



				//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
				//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
				//				output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;

				output_angle = gyro_angle;
				last_angle_error = now_angle_error;
				//				if(output_angle - raw_angle <5 && output_angle - raw_angle > -5){
				//					original_angle = 0.8*original_angle +0.2*raw_angle;
				//				}
				//				else if(output_angle - original_angle >=5){
				//					original_angle -= (output_angle - raw_angle-5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
				//				}
				//				else if(output_angle - original_angle <=-5){
				//					original_angle -= (output_angle - raw_angle + 5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
				//				}




				now_angle_error = output_angle - original_angle;
				angle_error_change = now_angle_error -last_angle_error;

				last_ideal_count = ideal_count;
				if(now_angle_error >= 0){
					square_sign = 1;
				}
				else if(now_angle_error < 0){
					square_sign = -1;
				}
				ideal_count = (int32_t)(ic_Kp*(now_angle_error)+ ic_Kd*angle_error_change);
				//			ideal_count = 3.1*(now_error)+ 0.9 * angle_error_change;

				//				if(last_ideal_count >=0){
				//					last_sign = 1;
				//				}
				//				else{
				//					last_sign = 0;
				//				}

				ideal_count = (int32_t)(0.5*last_ideal_count + 0.5*ideal_count);







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

				count_r = (int32_t)(-1*encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

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

					ir_encoder_errorsum -= ir_encoder_error;
					il_encoder_errorsum -= il_encoder_error;


					lincoln1 = (float)(ir_encoder_error_change * encoder_r_Kd/3);

					speed_r = (int32_t)(ir_encoder_error *encoder_r_Kp + (float)(ir_encoder_error_change * encoder_r_Kd/3) + ir_encoder_errorsum*encoder_r_Ki*0.003);
					speed_l = (int32_t)(il_encoder_error *encoder_l_Kp + (float)(il_encoder_error_change * encoder_l_Kd/3) + il_encoder_errorsum*encoder_l_Ki*0.003);
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




					speed_r = ratio_old*old_speed_r + ratio_new*speed_r;
					speed_l = ratio_old*old_speed_l + ratio_new*speed_l;

					speed_r = speed_r*turn[1];
					speed_l = speed_l*turn[0];


					motor_l.SetPower(abs(speed_l)+107);
					motor_r.SetPower(abs(speed_r)+113);


				}


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

				mpu6050.Update();

				accel = mpu6050.GetAccel();
				omega = mpu6050.GetOmega();

				last_accel_angle = accel_angle;
				accel_angle = accel[0]*57.29578;
				accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;

				last_gyro_angle = gyro_angle;
				gyro_angle += (-1) *omega[1]*0.005+0.006706 + trust_accel*(accel_angle - gyro_angle);
				//			gyro_angle = accel_angle+(-1) *omega[0];
				//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;



				//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
				//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
				//				output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;

				output_angle = gyro_angle;
				last_angle_error = now_angle_error;
				//				if(output_angle - raw_angle <5 && output_angle - raw_angle > -5){
				//					original_angle = 0.8*original_angle +0.2*raw_angle;
				//				}
				//				else if(output_angle - original_angle >=5){
				//					original_angle -= (output_angle - raw_angle-5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
				//				}
				//				else if(output_angle - original_angle <=-5){
				//					original_angle -= (output_angle - raw_angle + 5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
				//				}






				now_angle_error = output_angle - original_angle;
				angle_error_change = now_angle_error -last_angle_error;

				last_ideal_count = ideal_count;
				if(now_angle_error >= 0){
					square_sign = 1;
				}
				else if(now_angle_error < 0){
					square_sign = -1;
				}
				ideal_count = (int32_t)(ic_Kp*(now_angle_error)+ ic_Kd*angle_error_change);
				//			ideal_count = 3.1*(now_error)+ 0.9 * angle_error_change;

				//				if(last_ideal_count >=0){
				//					last_sign = 1;
				//				}
				//				else{
				//					last_sign = 0;
				//				}

				ideal_count = (int32_t)(0.5*last_ideal_count + 0.5*ideal_count);




			}





			if((int32_t)(t-pt3) >= 3 && yo == 3){
				//				lincoln.Turn();
				pt4 = System::Time();

				//				pt1 = t;
				yo = 4;


				encoder_r.Update();
				encoder_l.Update();

				count_r = (int32_t)(-1*encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

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

					ir_encoder_errorsum -= ir_encoder_error;
					il_encoder_errorsum -= il_encoder_error;


					lincoln1 = (float)(ir_encoder_error_change * encoder_r_Kd/3);

					speed_r = (int32_t)(ir_encoder_error *encoder_r_Kp + (float)(ir_encoder_error_change * encoder_r_Kd/3) + ir_encoder_errorsum*encoder_r_Ki*0.003);
					speed_l = (int32_t)(il_encoder_error *encoder_l_Kp + (float)(il_encoder_error_change * encoder_l_Kd/3) + il_encoder_errorsum*encoder_l_Ki*0.003);
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




					speed_r = ratio_old*old_speed_r + ratio_new*speed_r;
					speed_l = ratio_old*old_speed_l + ratio_new*speed_l;

					speed_r = speed_r*turn[1];
					speed_l = speed_l*turn[0];


					motor_l.SetPower(abs(speed_l)+107);
					motor_r.SetPower(abs(speed_r)+113);


				}


			}







			if((int32_t)(t-pt4) >= 2 && yo ==4){
				pt5 = System::Time();
				yo =0;
				pGrapher.sendWatchData();

			}




		}
	}




}








//		console.SetCursorRow(2);
//		console.PrintString(libutil::String::Format("Omega: %.3f", omega[2]).c_str(), -1);
//
//		console.SetCursorRow(4);
//		console.PrintString(libutil::String::Format("Accel: %.3f", accel[2]).c_str(), -1);
//
//		console.SetCursorRow(6);
//		console.PrintString(libutil::String::Format("Encoder:%.3d", count_l).c_str(), -1);

//		int n = sprintf(buffer, "%.3f,%.3f,",real_angle,peter_angle);
//		fu.SendBuffer((Byte*)buffer,n);
//		memset(buffer, 0, n);





//		motor_l.SetClockwise(1);
//		motor_r.SetClockwise(0);
//		motor_l.SetPower(a+130);
//		motor_r.SetPower(a+150);

//		console.SetCursorRow((uint8_t)2);
//		sprintf(buffer, "angle:%.2f,%.2f\nspeed:%d, slope:%f",real_angle,(angle[2]*360)/6.2831852,speed,slope);
//		console.PrintString((char*)buffer);
