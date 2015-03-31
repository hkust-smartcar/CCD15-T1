/*
 * lincoln_main.h
 *
 *  Created on: 2 Mar, 2015
 *      Author: lincoln
 */

#include <vector>
#include <libbase/k60/mcg.h>
#include <libsc/led.h>
#include <libsc/system.h>
#include <libsc/k60/ftdi_ft232r.h>
#include <libsc/alternate_motor.h>
#include <libsc/tower_pro_mg995.h>
#include <libsc/mpu6050.h>
#include <libsc/encoder.h>
#include <libsc/dir_motor.h>
#include <libsc/mma8451q.h>
#include <libsc/device_h/mma8451q.h>
#include <cstdio>
#include <libsc/tsl1401cl.h>
#include "libsc/st7735r.h"
#include <libsc/lcd_console.h>
#include <libsc/lcd_typewriter.h>
#include <libbase/k60/adc.h>
#include <libsc/joystick.h>
#include <libsc/dir_encoder.h>
#include <libutil/string.h>
#include <libutil/kalman_filter.h>

#include "libsc/k60/jy_mcu_bt_106.h"
#include "MyVarManager.h"

#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define aabbss(v) ((v > 0)? v : -v)
#define white_black     0
#define black_white     1
#define half_black       2
#define half_white       3

char CCD;
using namespace libsc;
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
	config.core_clock_khz = 150000;
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
float ic_Kd = 6.85;
float ic_Kp = 0;
float ic_Kp_const = 5.9;
float ic_Ki = 0;
float mg = 0;

float gyro_Ki = 0;
//float encoder_Kp = 204;
//float encoder_Kd = 81;
float encoder_r_Kp = 2.1;
float encoder_r_Kd = 9.82;
float encoder_r_Ki = 0;
float encoder_l_Kp = 1.017;
float encoder_l_Kd = 4.22;
float encoder_l_Ki = 0;

float turning_Kp = 0;
float turning_Kd = 0;
float turning_Ki = 0;
int32_t turning_count = 0;

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
Byte l_edge = 0;
Byte r_edge = 0;
Byte l_color_flag = 0;
Byte r_color_flag = 0;
float moving_gain = 0;
Byte blue_flag = 0;


//float howard =0;
float lincoln1 = 0;

int32_t ideal_count = 0;
void myListener(const std::vector<Byte> &bytes)
{
	switch (bytes[0])
	{
	case '3':
		if(moving_gain >= 0.1){
			moving_gain -= 0.1;
		}
		break;
	case '4':
		moving_gain += 0.1;
		break;
	case '#':
		if(moving_gain >= 1){
			moving_gain -= 1;
		}
		break;
	case '$':
		moving_gain += 1;
		break;
	case '1':
		if(ideal_count >= 5){
			ideal_count -= 5;
		}
		break;
	case '2':
		ideal_count += 5;
		break;
	case '!':
		if(ideal_count >= 20){
			ideal_count -= 20;
		}
		break;
	case '@':
		ideal_count += 20;
		break;
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
	case '9':
		if(ic_Kp_const > 0.1){
			ic_Kp_const -= 0.1;
		}
		break;
	case '0':
		ic_Kp_const += 0.1;
		break;
	case '(':
		if(ic_Kp_const > 1){
			ic_Kp_const -= 1;
		}
		break;
	case ')':
		ic_Kp_const += 1;
		break;
	case 'z':
		if(turning_Kp > 0.1){
			turning_Kp -= 0.1;
		}
		break;
	case 'x':
		turning_Kp += 0.1;
		break;
	case 'Z':
		if(turning_Kp > 1){
			turning_Kp -= 1;
		}
		break;
	case 'X':
		turning_Kp += 1;
		break;
	case 'c':
		if(turning_Kd > 0.05){
			turning_Kd -= 0.05;
		}
		break;
	case 'v':
		turning_Kd += 0.05;
		break;
	case 'C':
		if(turning_Kd > 0.1){
			turning_Kd -= 0.1;
		}
		break;
	case 'V':
		turning_Kd += 0.1;
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
		if(encoder_l_Kp > 0.001){
			encoder_l_Kp -= 0.001;
		}
		break;
	case 'j':
		encoder_l_Kp += 0.001;
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
		if(encoder_l_Kd > 1){
			encoder_l_Kd -= 1;
		}
		break;
	case 'L':
		encoder_l_Kd += 1;
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
	uint16_t now_5pixel_value = 0;
	uint16_t last_5pixel_value = 0;
	float pixel_difference_sum = 0;
	float pixel_avg_difference = 0;
	uint32_t avg = 0;
	uint32_t all = 0;
	std::array<uint16_t,Tsl1401cl::kSensorW> pixel;
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



	//		FtdiFt232r::Config uart_config;
	//		uart_config.id = 0;
	//		uart_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	//		FtdiFt232r fu(uart_config);

	//	JyMcuBt106::Config uart_config;
	//	uart_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	//	uart_config.rx_irq_threshold = 7;
	//	uart_config.is_rx_irq_threshold_percentage = false;
	//	uart_config.tx_buf_size = 50;
	//	JyMcuBt106 fu(uart_config);

	//	 Initialize other things as necessary...


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


	Tsl1401cl ccd(0);
	Byte center_line = 0;
	Byte center_line_flag = 0;
	int16_t center_line_error = 0;
	float last_center_line_error = 0;
	float now_center_line_error = 0;
	float center_line_error_change = 0;
	float road_length = 0;
	float center_line_errorsum = 0;

	St7735r::Config config;
	config.is_revert = false;
	St7735r lcd(config);

	LcdConsole::Config yoyo;
	yoyo.bg_color = 0;
	yoyo.text_color = -1;
	yoyo.lcd = &lcd;
	LcdConsole console(yoyo);

	LcdTypewriter::Config typeconfig;
	typeconfig.lcd = &lcd;
	typeconfig.bg_color = 0;
	typeconfig.text_color = -1;
	typeconfig.is_text_wrap = true;
	LcdTypewriter type(typeconfig);




	Gpo::Config howard;
	howard.pin = Pin::Name::kPtd1;
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
	//	original_angle = raw_angle;
	original_angle = -5.01;

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

	uint32_t pt0 = 0;
	uint32_t pt1 = 0;
	uint32_t pt2 = 0;
	uint32_t pt3 = 0;
	uint32_t pt4 = 0;
	uint32_t pt5 = 0;
	uint32_t pt6 = 0;
	uint32_t pt7 = 0;


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
	MyVarManager pGrapher;
	//graph testing variable

	//	pGrapher.addWatchedVar(&ir_Kp, "float", sizeof(float), "1");
	//		pGrapher.addWatchedVar(&ir_Kd, "2");
	//		pGrapher.addWatchedVar(&il_Kp, "3");
	pGrapher.addWatchedVar(&ideal_count, "1");
	pGrapher.addWatchedVar(&turning_Kp, "2");
	pGrapher.addWatchedVar(&turning_Kd, "3");
	//	pGrapher.addWatchedVar(&ic_Ki, "4");
	//	pGrapher.addWatchedVar(&encoder_r_Kd, "5");
	//	pGrapher.addWatchedVar(&ir_encoder_errorsum, "6");
	pGrapher.addWatchedVar(&output_angle, "4");
	pGrapher.addWatchedVar(&count_l, "5");
	pGrapher.addWatchedVar(&count_r, "6");
	pGrapher.addWatchedVar(&moving_gain, "7");

	//	pGrapher.addWatchedVar(&speed_l, "5");

	//	pGrapher.addWatchedVar(&speed_r, "7");
	//	pGrapher.addWatchedVar(&ratio_old, "8");



	//	pGrapher.addWatchedVar(&kalman_value[0], "6");
	//	pGrapher.addWatchedVar(&kalman_value[1], "7");

	pGrapher.Init(&myListener);
	//					lincoln.Turn();

	while(1){

		if(t !=System::Time()){
			t = System::Time();
			//			if(t - pt1 <0 ||t - pt2 < 0 ||t- pt3 < 0){
			//				pt1 = 0;
			//				pt2 = 0;
			//				pt3 = 0;
			//			}


			//			if((int32_t)(t-pt0) >=50){
			//				pt0 = t;
			//
			//			}

			if((int32_t)(t-pt1) >= 11  && yo==0){
				//
				//
				//					lincoln.Turn();
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



				ic_Kp = moving_gain*aabbss(output_angle)/20 + ic_Kp_const;
				ideal_count = (int32_t)(ic_Kp*(now_angle_error)+ ic_Kd*angle_error_change);
				//			ideal_count = 3.1*(now_error)+ 0.9 * angle_error_change;

				//				if(last_ideal_count >=0){
				//					last_sign = 1;
				//				}
				//				else{
				//					last_sign = 0;
				//				}

				ideal_count = (int32_t)(0.1*last_ideal_count + 0.9*ideal_count);





			}













			if((int32_t)(t-pt1) >= 1 && yo ==1){
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

				else if(last_sign != sign){

					motor_l.SetPower(0);
					motor_r.SetPower(0);
					motor_l.SetClockwise(sign);
					motor_r.SetClockwise(sign);
				}

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count - turning_count - count_r;        //turning_count is positive when car needs to turn right
				last_il_encoder_error = il_encoder_error;                        //left wheel faster right wheel slower
				il_encoder_error = ideal_count + turning_count - count_l;        //turning_count is positive when car needs to turn right


				il_encoder_error_change = il_encoder_error -last_il_encoder_error;
				ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;

				old_speed_r = speed_r;
				old_speed_l = speed_l;

				ir_encoder_errorsum -= ir_encoder_error;
				il_encoder_errorsum -= il_encoder_error;


				lincoln1 = (float)(ir_encoder_error_change * encoder_r_Kd/3);


				//					if((il_encoder_error_change+ir_encoder_error_change)/2 >= 15){
				//						speed_l = 2.4988*ideal_count + 31.4593;                           //data from graph,reference power
				//						speed_r = 2.38436*ideal_count + 41.1278;
				//					}
				speed_r += (int32_t)((int32_t)(ir_encoder_error *encoder_r_Kp) + (int32_t)(ir_encoder_error_change * encoder_r_Kd) + ir_encoder_errorsum*encoder_r_Ki);
				speed_l += (int32_t)((int32_t)(il_encoder_error *encoder_l_Kp) + (int32_t)(il_encoder_error_change * encoder_l_Kd) + il_encoder_errorsum*encoder_l_Ki);
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



				//				motor_l.SetPower(abs(2.4988*speed_l + 31.4593));
				//				motor_r.SetPower(abs(2.38436*speed_r + 41.1278));


			}





			/*second round to get angle and encoder
			 *
			 *
			 *
			 *
			 */

			if((int32_t)(t-pt2) >= 1  && yo == 2){
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
				ic_Kp = moving_gain*aabbss(output_angle)/20 + ic_Kp_const;
				ideal_count = (int32_t)(ic_Kp*(now_angle_error)+ ic_Kd*angle_error_change);
				//			ideal_count = 3.1*(now_error)+ 0.9 * angle_error_change;

				//				if(last_ideal_count >=0){
				//					last_sign = 1;
				//				}
				//				else{
				//					last_sign = 0;
				//				}


				ideal_count = (int32_t)(0.1*last_ideal_count + 0.9*ideal_count);




			}





			if((int32_t)(t-pt3) >= 1 && yo == 3){
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

				else if(last_sign != sign){

					motor_l.SetPower(0);
					motor_r.SetPower(0);
					motor_l.SetClockwise(sign);
					motor_r.SetClockwise(sign);
				}

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count - turning_count - count_r;        //turning_count is positive when car needs to turn right
				last_il_encoder_error = il_encoder_error;                        //left wheel faster right wheel slower
				il_encoder_error = ideal_count + turning_count - count_l;        //turning_count is positive when car needs to turn right


				il_encoder_error_change = il_encoder_error -last_il_encoder_error;
				ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;

				old_speed_r = speed_r;
				old_speed_l = speed_l;

				ir_encoder_errorsum -= ir_encoder_error;
				il_encoder_errorsum -= il_encoder_error;


				//						lincoln1 = (float)(ir_encoder_error_change * encoder_r_Kd/3);


				speed_r += (int32_t)((int32_t)(ir_encoder_error *encoder_r_Kp) + (int32_t)(ir_encoder_error_change * encoder_r_Kd) + ir_encoder_errorsum*encoder_r_Ki);
				speed_l += (int32_t)((int32_t)(il_encoder_error *encoder_l_Kp) + (int32_t)(il_encoder_error_change * encoder_l_Kd) + il_encoder_errorsum*encoder_l_Ki);
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





				//				motor_l.SetPower(abs(2.4988*speed_l + 31.4593));                        //data from graph, encoder--->power mapping
				//				motor_r.SetPower(abs(2.38436*speed_r + 41.1278));


			}










			if(yo == 4 && blue_flag == 1){
				//				pt5 = System::Time();
				yo = 5;
				pGrapher.sendWatchData();
				blue_flag = 0;
				//				int n = sprintf(buffer, "%d , %d\n",power, count_r);
				//				fu.SendBuffer((Byte*)buffer,n);
				//				memset(buffer, 0, n);

			}
			else if(yo == 4){
				yo = 5;
				blue_flag = 1;
			}

			//detect edge method***************
			if((int32_t)(t-pt6) >= 50 && yo == 5){
				lincoln.Set(1);
				pt6 = System::Time();
				yo = 0;

				ccd.StartSample();
				while (!ccd.SampleProcess())
				{}
				pixel = ccd.GetData();

				now_5pixel_value = pixel[57] + pixel[58] + pixel[59] + pixel[60] + pixel[61];
				for(int i=62; i < 118; i = i+5){
					last_5pixel_value = now_5pixel_value;
					now_5pixel_value = pixel[i] + pixel[i+1] + pixel[i+2] + pixel[i+3] + pixel[i+4];
					pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5;

					if(i == 62){
						pixel_avg_difference = pixel_difference_sum/((i-57)/5);
						if(pixel_difference_sum >= 1500){
							pixel_avg_difference = 1;
						}

					}

					if(pixel_avg_difference >= 10){
						pixel_avg_difference = 1;
					}
					if((now_5pixel_value - last_5pixel_value)/5 < -150*aabbss(pixel_avg_difference)){
						r_edge = i;
						r_color_flag = white_black;
						pixel_difference_sum = 0;
						break;
					}


					else if((now_5pixel_value - last_5pixel_value)/5 > 150*aabbss(pixel_avg_difference)){
						r_edge = i;
						r_color_flag = black_white;
						pixel_difference_sum = 0;
						break;
					}
					else if(i == 117){
						if((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 > 60000){
							r_color_flag = half_white;
							pixel_difference_sum = 0;
							break;
						}
						else if((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 < 35000){
							r_color_flag = half_black;
							pixel_difference_sum = 0;
							break;
						}
					}
					pixel_avg_difference = pixel_difference_sum/((i-57)/5);
				}

				now_5pixel_value = pixel[65] + pixel[66] + pixel[67] + pixel[68] + pixel[69];
				for(int i = 69; i > 8; i = i-5){
					last_5pixel_value = now_5pixel_value;
					now_5pixel_value = pixel[i] + pixel[i-1] + pixel[i-2] + pixel[i-3] + pixel[i-4];
					pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5;

					if(i == 69){
						pixel_avg_difference = pixel_difference_sum/((74-i)/5);
						if(pixel_difference_sum >= 1500){
							pixel_avg_difference = 1;
						}
					}
					if(pixel_avg_difference >= 10){
						pixel_avg_difference = 1;
					}

					if((now_5pixel_value - last_5pixel_value)/5 < -150*aabbss(pixel_avg_difference)){
						l_edge = i;
						l_color_flag = white_black;
						pixel_difference_sum = 0;
						break;
					}


					else if((now_5pixel_value - last_5pixel_value)/5 > 150*aabbss(pixel_avg_difference)){
						l_edge = i;
						l_color_flag = black_white;
						pixel_difference_sum = 0;
						break;
					}
					else if(i == 9){

						if((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 > 60000){
							l_color_flag = half_white;
							pixel_difference_sum = 0;
							break;
						}
						else if((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 < 35000){
							l_color_flag = half_black;
							pixel_difference_sum = 0;
							break;
						}
						break;
					}
					pixel_avg_difference = pixel_difference_sum/((74 - i)/5);
				}

				if(center_line_flag == 0){
					road_length = r_edge - l_edge;
					center_line = (road_length)/2+l_edge;
					center_line_flag++;
				}

				last_center_line_error = now_center_line_error;
				now_center_line_error = center_line - ((r_edge - l_edge)/2+l_edge);         //if the error is positive, car need to turn right
				center_line_error_change = now_center_line_error - last_center_line_error;
				center_line_errorsum += now_center_line_error;
				if(center_line_errorsum > 1000){
					center_line_errorsum = 1000;
				}
				else if(center_line_errorsum < -1000){
					center_line_errorsum = -1000;
				}


				turning_count += (int32_t)(now_center_line_error*turning_Kp + center_line_error_change*turning_Kd + center_line_errorsum*turning_Ki);


				lincoln.Set(0);

				//
				//				if(now_center_line_error >=0){
				//					turn[0] = ccd_Kp*now_center_line_error + ccd_Kd*center_line_error_change;
				//					turn[1] =
				//				}
				//
			}
			else if(yo == 5){
				yo = 0;
			}
//			if((int32_t)(t-pt7) >= 100){
//				pt7 = System::Time();
//				libsc::St7735r::Rect rect_;
//
//				if(l_color_flag == white_black){
//					if(r_color_flag == white_black){
//						rect_.x = 0;
//						rect_.y = 130;
//						rect_.w = l_edge;
//						rect_.h = 16;
//						lcd.SetRegion(rect_);
//						lcd.FillColor(BLACK);
//
//						rect_.x = l_edge;
//						rect_.y = 130;
//						rect_.w = r_edge - l_edge;
//						rect_.h = 16;
//						lcd.SetRegion(rect_);
//						lcd.FillColor(WHITE);
//
//						rect_.x = r_edge;
//						rect_.y = 130;
//						rect_.w = 128 - r_edge;
//						rect_.h = 16;
//						lcd.SetRegion(rect_);
//						lcd.FillColor(BLACK);
//					}
//					else if(r_color_flag == half_white){
//						rect_.x = 0;
//						rect_.y = 130;
//						rect_.w = l_edge;
//						rect_.h = 16;
//						lcd.SetRegion(rect_);
//						lcd.FillColor(BLACK);
//
//						rect_.x = l_edge;
//						rect_.y = 130;
//						rect_.w = 128 - l_edge;
//						rect_.h = 16;
//						lcd.SetRegion(rect_);
//						lcd.FillColor(WHITE);
//					}
//				}
//				else if(l_color_flag == half_white){
//					if(r_color_flag == white_black){
//						rect_.x = 0;
//						rect_.y = 130;
//						rect_.w = r_edge;
//						rect_.h = 16;
//						lcd.SetRegion(rect_);
//						lcd.FillColor(WHITE);
//
//						rect_.x = r_edge;
//						rect_.y = 130;
//						rect_.w = 128 - r_edge;
//						rect_.h = 16;
//						lcd.SetRegion(rect_);
//						lcd.FillColor(BLACK);
//					}
//					else if(r_color_flag == half_white){
//						rect_.x = 0;
//						rect_.y = 130;
//						rect_.w = 128;
//						rect_.h = 16;
//						lcd.SetRegion(rect_);
//						lcd.FillColor(WHITE);
//					}
//				}
//			}

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
