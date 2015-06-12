/*
 * merge.cpp
 *
 *  Created on: 29 Apr, 2015
 *      Author: lincoln
 */
/*
 * main.h
 *
 *  Created on: 21 Mar, 2015
 *      Author: Howard
 */

#include <libbase/k60/mcg.h>
#include <libsc/led.h>
#include <libsc/system.h>
#include <libsc/alternate_motor.h>
#include <libsc/tower_pro_mg995.h>
#include <libsc/mpu6050.h>
#include <libsc/encoder.h>
#include <libsc/dir_motor.h>
#include <libsc/mma8451q.h>
#include <libsc/device_h/mma8451q.h>
#include <cstdio>
#include <math.h>
#include <libsc/tsl1401cl.h>
#include "libsc/st7735r.h"
#include <libsc/lcd_console.h>
#include <libsc/lcd_typewriter.h>
#include <libbase/k60/adc.h>
#include <libsc/joystick.h>
#include <libsc/dir_encoder.h>
#include <libutil/string.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/ab_encoder.h>
#include <libutil/remote_var_manager.h>
#include <kalman.h>
#include <libsc/button.h>
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
#define rad_to_degree    57.29578


char CCD;
using namespace libsc;
using namespace libsc::k60;
using namespace libutil;

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



float ic_Kp = 20;                   // Recommend 50
float ic_Kd = 35;            	    // Recommend 45

float is_Kp = 0.3;                  // Recommend 0.05
float is_Ki = 0.3;
float is_Kd = 0;
float speed_error_sum = 0;
float speed_count = 0;
float speed_error_sum_max = 100;

//float turn_Kp = 0.018;         // Recommend around 0.018
//float turn_Ki = 0;
//float turn_Kd = 0;

float encoder_r_Kp = 1.2;        // Recommend 2
float encoder_r_Ki = 0.0;   // Recommend 0.0008
float encoder_r_Kd = 0.0;
float encoder_l_Kp = 2;        // Recommend 2
float encoder_l_Ki = 0.00;   // Recommend 0.0008
float encoder_l_Kd = 0.0;

float turn[2] = { 1, 1 };
float turn_Kp = 0.1;
float turn_Kd = 0;
float turn_count_l = 0;
float turn_count_r = 0;
float combine_count_l =0;
float combine_count_r = 0;
int mid_point = 0;
uint16_t ccd_average = 0;

float balance_ratio = 1.0f;
float balance_error_ratio = 0;
float balance_divider = 15.0f;

float speed_ratio = 0;
float speed_error_ratio = 0;
float speed_divider = 20.0f;

float turn_ratio = 0;
float turn_error_ratio = 0;
float turn_divider = 1000000.0f;



float original_angle = 0;
float new_original_angle = 0;
float still_Ki = 0.000;
float ratio_old = 0;
float ratio_new = 1-ratio_old;
int16_t first_count = 0;

float trust_accel = 0.03;
float trust_old_accel = 0;
float trust_new_accel = 1- trust_old_accel;

float ideal_count_l = 0.0f;
float ideal_count_r = 0.0f;
float turn_error = 0;

int16_t count_l =0;
int16_t count_r =0;



float lincoln1 = 0;

float balance_count = 0;
float accumulate_balance_count;
//float ideal_speed = 10;
std::array<uint16_t, Tsl1401cl::kSensorW> Data;




//      omg variables
float accel_angle = original_angle;
float raw_accel_angle = 0;
float last_gyro_angle = original_angle;
float gyro_angle = original_angle;
float last_accel_angle = original_angle;
float output_angle = 0;            //karmen filtered
float gyro_in_time = 0.0034;

float total_count_l =0;
float total_count_r =0;
float last_angle_error = 0;
float now_angle_error = 0;
float angle_error_change = 0;

float last_speed_error = 0;
float speed_error = 0;
float speed_error_change = 0;

float last_il_encoder_error = 0;
float last_ir_encoder_error = 0;
float ir_encoder_error = 0;
float il_encoder_error = 0;
float ir_encoder_error_change = 0;
float il_encoder_error_change = 0;
float last_ir_encoder_error_change = 0;
float last_il_encoder_error_change = 0;
float ir_encoder_errorsum = 0.0f;
float il_encoder_errorsum = 0.0f;
double kalman_value[2] = {0.1, -1.0};
Kalman acc(0.001f, kalman_value, 0, 1);
//	KF m_gyro_kf[3];
//	float kalman_value[2] = {0.3f, 1.5f};
//	kalman_filter_init(&m_gyro_kf[0], 0.01f, &kalman_value, original_angle, 1);
float speed_l = 0;                    //last output to motor left,0-1000
float speed_r = 0;                    //last output to motor right,0-1000

uint16_t pt0 = 0;
uint16_t pt1 = 0;
uint16_t pt2 = 0;
uint16_t pt3 = 0;
uint16_t pt4 = 0;
uint16_t pt5 = 0;
uint16_t pt6 = 0;
uint16_t pt7 = 0;
uint16_t pt8 = 0;


int16_t sign = 0;
int16_t last_sign = 0;
int16_t last_balance_count = 0;
//	int16_t ccd_counter = 2;
int16_t ccd_counter = 0;
bool print16_t_data = false;
int16_t white_count = 0;


int16_t moving_balance_count_1 = 0;
int16_t moving_balance_count_2 = 0;
int16_t moving_balance_count_3 = 0;
int16_t moving_balance_count_4 = 0;
int16_t moving_balance_count_5 = 0;
int16_t moving_balance_count_6 = 0;
Byte moving_count_flag = 0;

Byte moving_power_flag = 0;
float moving_ideal_count_l_1 = 0;
float moving_ideal_count_r_1 = 0;
float moving_ideal_count_l_2 = 0;
float moving_ideal_count_r_2 = 0;
float moving_ideal_count_l_3 = 0;
float moving_ideal_count_r_3 = 0;

Byte moving_speed_flag = 0;
float moving_speed_1 = 0;
float moving_speed_2 = 0;
float moving_speed_3 = 0;

Byte road_length_flag = 0;
int16_t road_length_1 = 0;
int16_t road_length_2 = 0;
int16_t road_length_3 = 0;
int16_t road_length_4 = 0;
int16_t road_length_5 = 0;
int16_t road_length_6 = 0;
int16_t road_length = 0;

float moving_accel_1 = 0;
float moving_accel_2 = 0;
float moving_accel_3 = 0;
float moving_accel_4 = 0;
float moving_accel_5 = 0;
float moving_accel_6 = 0;
float moving_accel_7 = 0;
float moving_accel_8 = 0;

Byte moving_accel_flag = 0;
int main()
{
	std::array<float, 3>accel;
	std::array<float, 3>angle;
	std::array<float, 3>omega;
	std::array<float, 3>raw_accel;

	//int16_tialize the system
	System::Init();
	Timer::TimerInt t = 0;
	Timer::TimerInt pt = t;
	pt = System::Time();

	//	RemoteVarManager* varmanager = new RemoteVarManager(4);

	//	Initalize the BT module
	//	JyMcuBt106::Config uart_config;
	//	uart_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	//	uart_config.rx_irq_threshold = 7;
	//	uart_config.is_rx_irq_threshold_percentage = false;
	//	uart_config.tx_buf_size = 50;
	//	JyMcuBt106 fu(uart_config);


	//	FtdiFt232r::Config uart_config;
	//	uart_config.id = 0;
	//	uart_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	//	FtdiFt232r fu(uart_config);

	char *buffer = new char[125]{0};

	//	libutil::InitDefaultFwriteHandler(&bt);

	//	RemoteVarManager::Var* turn_Kp = varmanager->Register("turn_Kp",RemoteVarManager::Var::Type::kReal);
	//	RemoteVarManager::Var* turn_Kd = varmanager->Register("turn_Kd",RemoteVarManager::Var::Type::kReal);
	//	RemoteVarManager::Var* ideal_speed = varmanager->Register("ideal_speed",RemoteVarManager::Var::Type::kReal);
	//	RemoteVarManager::Var* is_Kp = varmanager->Register("is_Kp",RemoteVarManager::Var::Type::kReal);

	printf("turn_Kp,real,0,0\n");
	printf("turn_Kd,real,1,0\n");
	printf("ideal_speed,real,2,0\n");
	printf("is_Kp,real,3,0\n");


	//********************************************************************************************************************
	Byte blue_flag =0;

	float ideal_speed = 10;
	float is_Kp = 0;
	float speed = 0;
	float speed_contribution = 0;
	float balance_contribution = 0;
	float turn_contribution_l = 0;
	float turn_contribution_r = 0;
	float angle_accel = 0;
	float angle_accel_const = 10;
	float ha_accel = 0;
	int16_t border_r = 0;
	int16_t border_l = 0;
	MyVarManager pGrapher;
	//graph testing variable


	//	pGrapher.addWatchedVar(&ideal_count_l,"ideal_count_l");
	//	pGrapher.addWatchedVar(&count_r,"count_r");
	//	pGrapher.addWatchedVar(&ir_encoder_error,"ir_encoder_error");
	pGrapher.addWatchedVar(&speed_error_sum,"speed_error_sum");
	pGrapher.addWatchedVar(&accel_angle,"accel_angle");
	pGrapher.addWatchedVar(&gyro_angle,"gyro_angle");
	pGrapher.addWatchedVar(&border_l,"border_l");
	pGrapher.addWatchedVar(&speed_count,"speed_count");
	pGrapher.addWatchedVar(&balance_count,"balance_count");
	pGrapher.addWatchedVar(&border_r,"border_r");






	//	pGrapher.addWatchedVar(&encoder_l_Kp);
	//	pGrapher.addWatchedVar(&encoder_r_Kp);





	pGrapher.addSharedVar(&speed_error_sum_max,"speed_error_sum_max");
	pGrapher.addSharedVar(&is_Ki,"is_Ki");
	pGrapher.addSharedVar(&is_Kp,"is_Kp");
	pGrapher.addSharedVar(&is_Kd,"is_Kd");
	pGrapher.addSharedVar(&ideal_speed,"ideal_speed");
	pGrapher.addSharedVar(&turn_Kp,"turn_Kp");
	pGrapher.addSharedVar(&turn_Kd,"turn_Kd");


	//		pGrapher.Init(&myListener);


















	Mpu6050::Config gyro_config;
	//sensitivity of gyro
	gyro_config.gyro_range = Mpu6050::Config::Range::kLarge;
	//sensitivity of accelerometer
	gyro_config.accel_range = Mpu6050::Config::Range::kLarge;
	gyro_config.cal_drift = true;
	Mpu6050 mpu6050(gyro_config);




	Mma8451q::Config accel_config;
	accel_config.id = 0;
	accel_config.power_mode = Mma8451q::Config::PowerMode::kLowNoiseLowPower;
	accel_config.output_data_rate = Mma8451q::Config::OutputDataRate::k200Hz;
	accel_config.i2c_master_ptr = mpu6050.GetI2cMaster();
	Mma8451q mma8451q(accel_config);

	double R[2] = {0.01, -1};

	Kalman Kalman_l(0.0001, R, 0, 1);
	Kalman Kalman_r(0.0001, R, 0, 1);

	Joystick::Config joycon;
	joycon.id = 0;
	joycon.is_active_low = true;
	Joystick joy(joycon);


	DirMotor::Config l_motor;
	l_motor.id = 1;
	DirMotor motor_l(l_motor);

	DirMotor::Config r_motor;
	r_motor.id = 0;
	DirMotor motor_r(r_motor);

	AbEncoder::Config enconfig;
	enconfig.id = 0;
	AbEncoder encoder_l(enconfig);

	AbEncoder::Config r_encoder;
	r_encoder.id = 1;
	AbEncoder encoder_r(r_encoder);

	Tsl1401cl ccd(0);

	St7735r::Config config1;
	config1.is_revert = false;
	St7735r lcd(config1);

	Button::Config b_config0;
	b_config0.id = 0;
	b_config0.is_active_low = true;
	Button button0(b_config0);

	Button::Config b_config1;
	b_config1.id = 1;
	b_config0.is_active_low = true;
	Button button1(b_config1);

	Led::Config led_config0;
	led_config0.id = 0;
	Led led0(led_config0);

	Led::Config led_config1;
	led_config1.id = 1;
	Led led1(led_config1);

	System::DelayMs(25);

	motor_l.SetPower(0);
	motor_r.SetPower(0);
	count_l = 0;
	count_r = 0;



	float raw_angle =0;
	t= System::Time();
	pt = t;
	while(1){

		mpu6050.Update();
		System::DelayMs(4);
		accel = mpu6050.GetAccel();
		raw_angle = -accel[0]*57.2958;

		t = System::Time();
		if((t-pt)>=3000)
			break;
	}

	original_angle = raw_angle;
	gyro_angle = raw_angle;





	Byte yo = 0;                      //to organize the sequence of code

	encoder_r.Update();               //to reset the count
	encoder_l.Update();

	encoder_r.Update();               //to reset the count
	encoder_l.Update();
	while(1){


		if(t !=System::Time()){
			t = System::Time();

			//			if(t - pt7 > 1000)
			//			{
			//				pt7 = t;
			//				if (button0.IsDown())
			//				{
			//					led0.Switch();
			//					switch(ccd_counter)
			//					{
			//					case 2:
			//						ccd_counter = 10;
			//						break;
			//					case 10:
			//						ccd_counter = 2;
			//						break;
			//					default:
			//						ccd_counter = 2;
			//						break;
			//					}
			//				}
			//			}

			if((int16_t)(t-pt1) >= 5  && yo==0){
				pt1 = System::Time();
				yo = 1;


				if(blue_flag == 0){
					pGrapher.sendWatchData();
										int16_t n = sprintf(buffer, "ha \n");
										fu.SendBuffer((Byte*)buffer,n);
										memset(buffer, 0, n);
					//										pGrapher.sendWatchData();

					blue_flag = 1;
				}
				else if(blue_flag ==1){
					blue_flag =0;
				}
				//				else if(blue_flag ==2){
				//					blue_flag = 0;
				//				}

				encoder_r.Update();
				encoder_l.Update();



				//				total_count_l += (float)count_l * 0.001;
				//				total_count_r += (float)count_r * 0.001;
				//
				//				last_ir_encoder_error = ir_encoder_error;
				//				ir_encoder_error = balance_count - count_r;
				//				last_il_encoder_error = il_encoder_error;
				//				il_encoder_error = balance_count - count_l;



				//				if(turn[0] >= 0.8 && turn [1] >= 0.8)
				//					speed = (count_l + count_r)/ 2;
				//				else if(turn[0] == 1)
				//					speed = count_l;
				//				else if(turn[1] == 1)
				//					speed = count_r;
				//
				//				last_speed_error = speed_error;
				//				speed_error = ideal_speed - speed;

				//				ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
				//				il_encoder_errorsum += (float)il_encoder_error * 0.001;


				mpu6050.Update();
				mma8451q.Update();

				accel = mma8451q.GetAccel();
				omega = mpu6050.GetOmega();

				// qian qing angle reduce

				raw_accel = mma8451q.GetAccel();
				raw_accel_angle = raw_accel[1] * 57.29578;
				last_accel_angle = accel_angle;
				ha_accel = accel[1] * 57.29578;
				double temp = 0;
				acc.Filtering(&temp, (double)accel[1], 0);
				accel[1] = temp;
				accel_angle = (float)(accel[1]*57.29578);

				//				accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;

				//				if(moving_accel_flag == 0){
				//					moving_accel_1 = accel_angle;
				//					moving_accel_flag = 1;
				//				}
				//				else if(moving_accel_flag == 1){
				//					moving_accel_2 = accel_angle;
				//					moving_accel_flag = 2;
				//				}
				//				else if(moving_accel_flag == 2){
				//					moving_accel_3 = accel_angle;
				//					moving_accel_flag = 3;
				//				}
				//				else if(moving_accel_flag ==3){
				//					moving_accel_4 = accel_angle;
				//					moving_accel_flag = 0;
				//				}


				//				else if(moving_accel_flag == 4){
				//					moving_accel_5 = accel_angle;
				//					moving_accel_flag = 5;
				//				}
				//				else if(moving_accel_flag == 5){
				//					moving_accel_6 = accel_angle;
				//					moving_accel_flag = 6;
				//				}
				//				else if(moving_accel_flag == 6){
				//					moving_accel_7 = accel_angle;
				//					moving_accel_flag = 7;
				//				}
				//				else if(moving_accel_flag ==7){
				//					moving_accel_8 = accel_angle;
				//					moving_accel_flag = 0;
				//				}
				//				if(moving_accel_1 && moving_accel_2 && moving_accel_3 && moving_accel_4){  //  && moving_accel_7 && moving_accel_8
				//					accel_angle = (moving_accel_1 + moving_accel_2 + moving_accel_3 + moving_accel_4)/4.0f;//  + moving_accel_7 + moving_accel_8
				//				}
				last_gyro_angle = gyro_angle;
				gyro_angle += omega[1] * gyro_in_time + trust_accel * (accel_angle - gyro_angle);
				angle_accel = (gyro_angle - last_gyro_angle)*angle_accel_const;
				//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
				//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
				//				output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;









				output_angle = gyro_angle;
				last_angle_error = now_angle_error;
				now_angle_error = original_angle - output_angle;
				angle_error_change = now_angle_error -last_angle_error;

				//				accumulate_balance_count += ic_Kp * now_angle_error  + ic_Kd * angle_error_change / 0.03 - still_Ki * total_count_r;
				//				balance_count = ideal_speed + accumulate_balance_count;





				//				if(speed > ideal_speed){
				//					if(now_angle_error >= 0){
				//						accumulate_balance_count += ic_Kp * now_angle_error* now_angle_error  + ic_Kd * angle_error_change / 0.03 - still_Ki * total_count_r;
				//						balance_count = ideal_speed + accumulate_balance_count;
				//					}
				//					else{
				//						accumulate_balance_count += ic_Kp * now_angle_error* now_angle_error  + ic_Kd * angle_error_change / 0.03 - still_Ki * total_count_r;
				//						balance_count = ideal_speed + accumulate_balance_count;
				//					}
				//				}
				//				else if(speed < ideal_speed){
				//					if(now_angle_error >= 0){
				//
				//					}
				//					else{
				//
				//					}
				//				}



				balance_count = ideal_speed + ic_Kp * now_angle_error  + ic_Kd * angle_error_change - still_Ki * total_count_r  - angle_accel;
				//				balance_count = (int16_t)(0.2*last_balance_count + 0.8*balance_count);

			}



			if((int16_t)(t-pt1) >= 1 && yo ==1){
				//				lincoln.Turn();
				pt2=System::Time();

				//				pt1 = t;
				yo = 2;

				encoder_r.Update();
				encoder_l.Update();

				count_r = (int16_t)(-encoder_r.GetCount());
				count_l = (int16_t)(encoder_l.GetCount());

				//				double temp = 0;
				//				Kalman_l.Filtering(&temp, count_l, 0.0);
				//				count_l = temp;
				//				Kalman_r.Filtering(&temp, count_r, 0.0);
				//				count_r = temp;

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;





				//				if(turn[0] >= 0.8 && turn [1] >= 0.8)
				speed = (count_l + count_r)/ 2.0f;
				//				//				else if(turn[0] == 1)
				//				//					speed = count_l;
				//				//				else if(turn[1] == 1)
				//				//					speed = count_r;
				//
				last_speed_error = speed_error;
				speed_error = speed - ideal_speed;
				speed_error_sum += speed_error;
				if(speed_error_sum >= speed_error_sum_max){
					speed_error_sum = speed_error_sum_max;
				}
				if(speed_error_sum <= -speed_error_sum_max){
					speed_error_sum = -speed_error_sum_max;
				}
				speed_error_change = (speed_error - last_speed_error);

				//				original_angle = raw_angle - is_Kp * speed_error - is_Kd * speed_error_change;
				speed_count = is_Kp * speed_error + is_Kd * speed_error_change + is_Ki * speed_error_sum;

				if(raw_angle - output_angle > 10 || raw_angle - output_angle < -10){
					speed_count = 0;
				}





				//				if(moving_speed_flag == 0){
				//					moving_speed_1 = speed_count;
				//					moving_speed_flag = 1;
				//				}
				//				else if(moving_speed_flag == 1){
				//					moving_speed_2 = speed_count;
				//					moving_speed_flag = 2;
				//				}
				//				else if(moving_speed_flag == 2){
				//					moving_speed_3 = speed_count;
				//					moving_speed_flag = 0;
				//				}
				//
				//
				//
				//				speed_count = (moving_speed_1 + moving_speed_2 + moving_speed_3)/3.0f;


				//				turn_count_l = (balance_count) * turn[0];
				//				turn_count_r = (balance_count) * turn[1];


				//				balance_error_ratio = aabbss(now_angle_error)/balance_divider;
				//				speed_error_ratio = aabbss(speed_error)/speed_divider;
				//				turn_error_ratio = aabbss(turn_error)/turn_divider;
				//
				//				balance_ratio = balance_error_ratio/(balance_error_ratio + speed_error_ratio + turn_error_ratio);
				//				//				speed_ratio = speed_error_ratio/(balance_error_ratio + speed_error_ratio + turn_error_ratio);
				//				turn_ratio = turn_error_ratio/(balance_error_ratio + speed_error_ratio + turn_error_ratio);
				//
				//				ideal_count_l = balance_ratio * balance_count + turn_ratio * turn_count_l;
				//				ideal_count_r = balance_ratio * balance_count + turn_ratio * turn_count_r;
				//
				//
				//				combine_count_l = ideal_count_l;
				//				combine_count_r = ideal_count_r;
				//
				//				speed_contribution = speed_ratio * speed_count;
				//				balance_contribution = balance_ratio * balance_count;
				//				turn_contribution_l = turn_ratio * turn_count_l;
				//				turn_contribution_r = turn_ratio * turn_count_r;
				ideal_count_l = (balance_count + speed_count) * turn[0];
				ideal_count_r = (balance_count + speed_count) * turn[1];


				//				ideal_count_l = ideal_count_l * turn[0];
				//				ideal_count_r = ideal_count_r * turn[1];





				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count_r - count_r;
				last_il_encoder_error = il_encoder_error;
				il_encoder_error = ideal_count_l - count_l;

				//				    if(turn[0] >= 0.8 && turn [1] >= 0.8)
				//					speed = (count_l + count_r)/ 0.002;
				//				    else if(turn[0] == 1)
				//				    	speed = count_l / 0.001;
				//				    else if(turn[1] == 1)
				//				    	speed = count_r / 0.001;

				//					last_speed_error = speed_error;
				//					speed_error = ideal_speed - speed;

				ir_encoder_errorsum += ir_encoder_error * 0.001;
				il_encoder_errorsum += il_encoder_error * 0.001;

				ideal_count_r = ideal_count_r + ir_encoder_error * encoder_r_Kp;// + ir_encoder_errorsum * encoder_r_Ki;
				ideal_count_l = ideal_count_l + il_encoder_error * encoder_l_Kp;// + il_encoder_errorsum * encoder_l_Ki;












				//				if(moving_power_flag == 0){
				//					moving_ideal_count_l_1 = ideal_count_l;
				//					moving_ideal_count_r_1 = ideal_count_r;
				//					moving_power_flag = 1;
				//				}
				//				else if(moving_power_flag == 1){
				//					moving_ideal_count_l_2 = ideal_count_l;
				//					moving_ideal_count_r_2 = ideal_count_r;
				//					moving_power_flag = 2;
				//				}
				//				else if(moving_power_flag == 2){
				//					moving_ideal_count_l_3 = ideal_count_l;
				//					moving_ideal_count_r_3 = ideal_count_r;
				//					moving_power_flag = 0;
				//				}
				//
				//
				//
				//				ideal_count_l = (moving_ideal_count_l_1 + moving_ideal_count_l_2 + moving_ideal_count_l_3)/3.0f;
				//				ideal_count_r = (moving_ideal_count_r_1 + moving_ideal_count_r_2 + moving_ideal_count_r_3)/3.0f;






				speed_l = 1.65479f * ideal_count_l;
				speed_r = 1.90774f * ideal_count_r;




				last_sign = sign;
				if((speed_l + speed_r)/2 > 0){
					sign = 0;
				}
				else if((speed_l + speed_r)/2 < 0){
					sign = 1;
				}
				//				else if((speed_l + speed_r)/2 == 0){
				//					sign = 2;
				//				}


				//				if(sign == 2){
				//					motor_l.SetPower(0);
				//					motor_r.SetPower(0);
				//				}



				if(last_sign != sign){
					motor_l.SetPower(0);
					motor_r.SetPower(0);
					motor_l.SetClockwise(sign);
					motor_r.SetClockwise(sign);
				}
				else {
					motor_l.SetPower(0);
					motor_r.SetPower(0);
					motor_l.SetClockwise(sign);
					motor_r.SetClockwise(sign);
				}












				speed_r = (aabbss(speed_r) + 40.0f);
				speed_l = (aabbss(speed_l) + 41.38f);



				if(speed_l > 500 && sign == 0){
					speed_l = 500;
				}
				else if(speed_l < -500 && sign ==1){
					speed_l = -500;
				}


				if(speed_r > 500 && sign ==0){
					speed_r = 500;
				}
				else if(speed_r < -500 && sign ==1){
					speed_r = -500;
				}

				motor_l.SetPower((int16_t)(speed_l));
				motor_r.SetPower((int16_t)(speed_r));



			}




			if((int16_t)(t-pt2) >= 1 && yo == 2){
				//				howard.Set(1);
				pt0 = System::Time();
				yo = 8;

				//				encoder_r.Update();
				//				encoder_l.Update();
				//
				//				count_r = (int16_t)(-encoder_r.GetCount());
				//				count_l = (int16_t)(encoder_l.GetCount());



				//				total_count_l += (float)count_l * 0.001;
				//				total_count_r += (float)count_r * 0.001;
				//
				//				last_ir_encoder_error = ir_encoder_error;
				//				ir_encoder_error = balance_count - count_r;
				//				last_il_encoder_error = il_encoder_error;
				//				il_encoder_error = balance_count - count_l;

				//				if(turn[0] >= 0.8 && turn [1] >= 0.8)
				//					speed = (count_l + count_r)/ 2;
				//				else if(turn[0] == 1)
				//					speed = count_l;
				//				else if(turn[1] == 1)
				//					speed = count_r;

				//				last_speed_error = speed_error;
				//				speed_error = ideal_speed - speed;

				//				ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
				//				il_encoder_errorsum += (float)il_encoder_error * 0.001;

				// Clean old lcd pixels
				if(print16_t_data){
					St7735r::Rect rect_1;
					for(int16_t i = 0; i<Tsl1401cl::kSensorW; i++){
						rect_1.x = i;
						rect_1.y = Data[i];
						rect_1.w = 1;
						rect_1.h = 1;
						lcd.SetRegion(rect_1);
						lcd.FillColor(0);
					}
				}

				int16_t a = 63, b = 64;
				int16_t border_r = Tsl1401cl::kSensorW - 1, border_l = 0;

				while(!ccd.SampleProcess()){};
				Data = ccd.GetData();  // 0 - 127 is left to right from the view of CCD
				ccd.StartSample();
				uint16_t ccd_sum = 0;

				for(int16_t i = 0; i < Tsl1401cl::kSensorW; i++){
					Data[i] = (float)Data[i] * 4;
					ccd_sum += Data[i];
				}

				uint16_t ccd_average = ccd_sum / Tsl1401cl::kSensorW;

				if(ccd_average > 72)
					ccd_average = 72;
				else if(ccd_average < 35)
					ccd_average = 35;
				else
					ccd_average = ccd_average + 3;

				white_count = 0;

				for(int16_t i = 0; i < Tsl1401cl::kSensorW; i++){
					if(Data[i] < ccd_average)
						Data[i] = 0;
					else
					{
						Data[i] = 1;
						white_count++;
					}
				}


				// left is 0
				if(Data[63] == 1){
					for (a = a + 1; a < Tsl1401cl::kSensorW; a++)
					{
						if(Data[a] == 1)
							border_r = a;
						else
							break;
					}

					for(b = b - 1; b >= 0; b--)
					{
						if(Data[b] == 1)
							border_l = b;
						else
							break;
					}


					//						while(border_r - border_l < 80)
					//						{
					//							for (a = a + 1; a < Tsl1401cl::kSensorW; a++)
					//							{
					//								if(Data[a] == 60)
					//									border_r = a;
					//								else
					//									break;
					//							}
					//
					//							for(b = b - 1; b >= 0; b--)
					//							{
					//								if(Data[b] == 60)
					//									border_l = b;
					//								else
					//									break;
					//							}
					//						}
				}

				else
				{
					if(mid_point >= 63)
					{
						for(int16_t i = 64; i < Tsl1401cl::kSensorW; i++)
						{
							if (Data[i] == 1)
							{
								border_l = i;
								border_r = Tsl1401cl::kSensorW - 1;
								break;
							}
							else{
								border_l = i;
								border_r = i;
							}
						}
					}

					else
					{
						for(int16_t i = 62; i >= 0; i--)
						{
							if (Data[i] == 1)
							{
								border_r = i;
								border_l = 0;
								break;
							}
							else{
								border_l = i;
								border_r = i;
							}
						}
					}
				}


				if(road_length_flag == 0){
					road_length_1 = border_r - border_l;
					road_length_flag = 1;
				}
				else if(road_length_flag == 1){
					road_length_2 = border_r - border_l;
					road_length_flag = 2;
				}
				else if(road_length_flag == 2){
					road_length_3 = border_r - border_l;
					road_length_flag = 3;
					road_length = (int16_t)((road_length_1 + road_length_2 + road_length_3)/3.0f);
				}










				if(border_r - border_l - road_length < 10){
					mid_point = (border_l + border_r) / 2;
					if(border_l <= 5){
						mid_point = (2 * border_r - road_length)/2;
					}
					else if(border_r >= 123){
						mid_point = (2 * border_l + road_length)/2;
					}
				}


				else if(border_r - border_l - road_length > 10)
				{
					if(border_l <= 20){
						mid_point = (2 * border_r - road_length)/2 - 10;
					}
					else if(border_r >= 120){
						mid_point = (2 * border_l + road_length)/2 + 2;
					}
					else if(border_l <= 15 && border_r >= 117){
						mid_point = (border_l + border_r) / 2;
					}

				}
				float last_turn_error = turn_error;
				turn_error = 63 - mid_point;
				turn[0] = 1;
				turn[1] = 1;

				float turn_error_change = turn_error - last_turn_error;
				float hehe = turn_Kp;
				//				if(white_count >= 105)
				//					hehe = hehe / 2;

				if(turn_error > 0){
					turn[0] = 1 - hehe * turn_error - turn_error_change * turn_Kd;
					//					turn[0] -= hehe * turn_error - turn_error_change * turn_Kd;
					if(turn[0] < 0 || turn[0] > 2){
						if(turn[0] < 0){
							turn[0] = 0;
						}
						else{
							turn[0] = 2;
						}
					}
				}
				else{
					turn[1] = 1 + hehe * 0.75 * turn_error + turn_error_change * turn_Kd;
					//					turn[1] += hehe * turn_error + turn_error_change * turn_Kd;
					if(turn[1] < 0 || turn[1] > 2){
						if(turn[1] < 0){
							turn[1] = 0;
						}
						else{
							turn[1] = 2;
						}
					}
				}


				if(print16_t_data){
					St7735r::Rect rect_1;
					for(int16_t i = 0; i<Tsl1401cl::kSensorW; i++){
						rect_1.x = i;
						rect_1.y = Data[i];
						rect_1.w = 1;
						rect_1.h = 1;
						lcd.SetRegion(rect_1);
						lcd.FillColor(~0);
					}
				}


			}



			/*second round to get angle and encoder
			 *
			 *
			 *
			 *
			 */
			if((int16_t)(t-pt0) >= 2  && yo == 8){
				pt3 = System::Time();

				yo = 3;

				encoder_r.Update();
				encoder_l.Update();

				//				count_r = (int16_t)(-encoder_r.GetCount());
				//				count_l = (int16_t)(encoder_l.GetCount());
				//
				//				last_ir_encoder_error = ir_encoder_error;
				//				ir_encoder_error = balance_count - (float)count_r;
				//				last_il_encoder_error = il_encoder_error;
				//				il_encoder_error = balance_count - (float)count_l;
				//
				//				total_count_l += (float)count_l * 0.001;
				//				total_count_r += (float)count_r * 0.001;
				//
				//				//					if(turn[0] >= 0.8 && turn [1] >= 0.8)
				//				//						speed = (count_l + count_r)/ 2;
				//				//					else if(turn[0] == 1)
				//				//						speed = count_l;
				//				//					else if(turn[1] == 1)
				//				//						speed = count_r;
				//				//
				//				//					last_speed_error = speed_error;
				//				//					speed_error = ideal_speed - speed;
				//
				//				ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
				//				il_encoder_errorsum += (float)il_encoder_error * 0.001;


				mpu6050.Update();
				mma8451q.Update();

				accel = mma8451q.GetAccel();
				omega = mpu6050.GetOmega();

				// qian qing angle reduce

				raw_accel = mma8451q.GetAccel();
				raw_accel_angle = raw_accel[1] * 57.29578;
				last_accel_angle = accel_angle;
				double temp = 0;
				acc.Filtering(&temp, (double)accel[1], 0);
				accel[1] = temp;
				accel_angle = (float)(accel[1]*57.29578);				//				accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;

				//				if(moving_accel_flag == 0){
				//					moving_accel_1 = accel_angle;
				//					moving_accel_flag = 1;
				//				}
				//				else if(moving_accel_flag == 1){
				//					moving_accel_2 = accel_angle;
				//					moving_accel_flag = 2;
				//				}
				//				else if(moving_accel_flag == 2){
				//					moving_accel_3 = accel_angle;
				//					moving_accel_flag = 3;
				//				}
				//				else if(moving_accel_flag ==3){
				//					moving_accel_4 = accel_angle;
				//					moving_accel_flag = 0;
				//				}


				//				else if(moving_accel_flag == 4){
				//					moving_accel_5 = accel_angle;
				//					moving_accel_flag = 5;
				//				}
				//				else if(moving_accel_flag == 5){
				//					moving_accel_6 = accel_angle;
				//					moving_accel_flag = 6;
				//				}
				//				else if(moving_accel_flag == 6){
				//					moving_accel_7 = accel_angle;
				//					moving_accel_flag = 7;
				//				}
				//				else if(moving_accel_flag ==7){
				//					moving_accel_8 = accel_angle;
				//					moving_accel_flag = 0;
				//				}
				//				if(moving_accel_1 && moving_accel_2 && moving_accel_3 && moving_accel_4){  //  && moving_accel_7 && moving_accel_8
				//					accel_angle = (moving_accel_1 + moving_accel_2 + moving_accel_3 + moving_accel_4)/4.0f;//  + moving_accel_7 + moving_accel_8
				//				}
				last_gyro_angle = gyro_angle;
				gyro_angle += omega[1] * gyro_in_time + trust_accel * (accel_angle - gyro_angle);
				angle_accel = (gyro_angle - last_gyro_angle)*angle_accel_const;
				//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;


				//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
				//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
				//				output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;

				output_angle = gyro_angle;
				last_angle_error = now_angle_error;
				now_angle_error = original_angle - output_angle;
				angle_error_change = now_angle_error -last_angle_error;



				balance_count = ideal_speed + ic_Kp * now_angle_error  + ic_Kd * angle_error_change - still_Ki * total_count_r  - angle_accel;
				//				balance_count = (int16_t)(0.2*last_balance_count + 0.8*balance_count);
				//				if(moving_count_flag == 0){
				//					moving_balance_count_1 = balance_count;
				//					moving_count_flag = 1;
				//				}
				//				else if(moving_count_flag == 1){
				//					moving_balance_count_2 = balance_count;
				//					moving_count_flag = 2;
				//				}
				//				else if(moving_count_flag == 2){
				//					moving_balance_count_3 = balance_count;
				//					moving_count_flag = 0;
				//				}
				//				//				else if(moving_count_flag ==3){
				//				//					moving_balance_count_4 = balance_count;
				//				//					moving_count_flag = 0;
				//				//				}
				//
				//				if(moving_balance_count_1 && moving_balance_count_2 && moving_balance_count_3){
				//					balance_count = (int16_t)((moving_balance_count_1 + moving_balance_count_2 + moving_balance_count_3)/3.0f);
				//				}

			}


			if((int16_t)(t-pt3) >= 1 && yo == 3){
				//				lincoln.Turn();
				pt4 = System::Time();

				//				pt1 = t;
				yo = 0;


				encoder_r.Update();
				encoder_l.Update();

				count_r = (int16_t)(-encoder_r.GetCount());
				count_l = (int16_t)(encoder_l.GetCount());

				//				double temp = 0;
				//				Kalman_l.Filtering(&temp, count_l, 0.0);
				//				count_l = temp;
				//				Kalman_r.Filtering(&temp, count_r, 0.0);
				//				count_r = temp;

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;





				//				if(turn[0] >= 0.8 && turn [1] >= 0.8)
				speed = (count_l + count_r)/ 2.0f;
				//				//				else if(turn[0] == 1)
				//				//					speed = count_l;
				//				//				else if(turn[1] == 1)
				//				//					speed = count_r;
				//
				last_speed_error = speed_error;
				speed_error = speed - ideal_speed;
				speed_error_sum += speed_error;
				if(speed_error_sum >= speed_error_sum_max){
					speed_error_sum = speed_error_sum_max;
				}
				if(speed_error_sum <= -speed_error_sum_max){
					speed_error_sum = -speed_error_sum_max;
				}
				speed_error_change = (speed_error - last_speed_error);

				//				original_angle = raw_angle - is_Kp * speed_error - is_Kd * speed_error_change;
				speed_count = is_Kp * speed_error + is_Kd * speed_error_change + is_Ki * speed_error_sum;

				if(raw_angle - output_angle > 10 || raw_angle - output_angle < -10){
					speed_count = 0;
				}





				//				if(moving_speed_flag == 0){
				//					moving_speed_1 = speed_count;
				//					moving_speed_flag = 1;
				//				}
				//				else if(moving_speed_flag == 1){
				//					moving_speed_2 = speed_count;
				//					moving_speed_flag = 2;
				//				}
				//				else if(moving_speed_flag == 2){
				//					moving_speed_3 = speed_count;
				//					moving_speed_flag = 0;
				//				}
				//
				//
				//
				//				speed_count = (moving_speed_1 + moving_speed_2 + moving_speed_3)/3.0f;


				//				turn_count_l = (balance_count) * turn[0];
				//				turn_count_r = (balance_count) * turn[1];


				//				balance_error_ratio = aabbss(now_angle_error)/balance_divider;
				//				speed_error_ratio = aabbss(speed_error)/speed_divider;
				//				turn_error_ratio = aabbss(turn_error)/turn_divider;
				//
				//				balance_ratio = balance_error_ratio/(balance_error_ratio + speed_error_ratio + turn_error_ratio);
				//				//				speed_ratio = speed_error_ratio/(balance_error_ratio + speed_error_ratio + turn_error_ratio);
				//				turn_ratio = turn_error_ratio/(balance_error_ratio + speed_error_ratio + turn_error_ratio);
				//
				//				ideal_count_l = balance_ratio * balance_count + turn_ratio * turn_count_l;
				//				ideal_count_r = balance_ratio * balance_count + turn_ratio * turn_count_r;
				//
				//
				//				combine_count_l = ideal_count_l;
				//				combine_count_r = ideal_count_r;
				//
				//				speed_contribution = speed_ratio * speed_count;
				//				balance_contribution = balance_ratio * balance_count;
				//				turn_contribution_l = turn_ratio * turn_count_l;
				//				turn_contribution_r = turn_ratio * turn_count_r;
				ideal_count_l = balance_count + speed_count * turn[0];
				ideal_count_r = (balance_count + speed_count) * turn[1];


				//				ideal_count_l = ideal_count_l * turn[0];
				//				ideal_count_r = ideal_count_r * turn[1];





				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count_r - count_r;
				last_il_encoder_error = il_encoder_error;
				il_encoder_error = ideal_count_l - count_l;

				//				    if(turn[0] >= 0.8 && turn [1] >= 0.8)
				//					speed = (count_l + count_r)/ 0.002;
				//				    else if(turn[0] == 1)
				//				    	speed = count_l / 0.001;
				//				    else if(turn[1] == 1)
				//				    	speed = count_r / 0.001;

				//					last_speed_error = speed_error;
				//					speed_error = ideal_speed - speed;

				ir_encoder_errorsum += ir_encoder_error * 0.001;
				il_encoder_errorsum += il_encoder_error * 0.001;

				ideal_count_r = ideal_count_r + ir_encoder_error * encoder_r_Kp + ir_encoder_errorsum * encoder_r_Ki;
				ideal_count_l = ideal_count_l + il_encoder_error * encoder_l_Kp + il_encoder_errorsum * encoder_l_Ki;












				//				if(moving_power_flag == 0){
				//					moving_ideal_count_l_1 = ideal_count_l;
				//					moving_ideal_count_r_1 = ideal_count_r;
				//					moving_power_flag = 1;
				//				}
				//				else if(moving_power_flag == 1){
				//					moving_ideal_count_l_2 = ideal_count_l;
				//					moving_ideal_count_r_2 = ideal_count_r;
				//					moving_power_flag = 2;
				//				}
				//				else if(moving_power_flag == 2){
				//					moving_ideal_count_l_3 = ideal_count_l;
				//					moving_ideal_count_r_3 = ideal_count_r;
				//					moving_power_flag = 0;
				//				}
				//
				//
				//
				//				ideal_count_l = (moving_ideal_count_l_1 + moving_ideal_count_l_2 + moving_ideal_count_l_3)/3.0f;
				//				ideal_count_r = (moving_ideal_count_r_1 + moving_ideal_count_r_2 + moving_ideal_count_r_3)/3.0f;






				speed_l = 1.65479f * ideal_count_l;
				speed_r = 1.90774f * ideal_count_r;




				last_sign = sign;
				if((speed_l + speed_r)/2 > 0){
					sign = 0;
				}
				else if((speed_l + speed_r)/2 < 0){
					sign = 1;
				}
				//				else if((speed_l + speed_r)/2 == 0){
				//					sign = 2;
				//				}


				//				if(sign == 2){
				//					motor_l.SetPower(0);
				//					motor_r.SetPower(0);
				//				}



				if(last_sign != sign){
					motor_l.SetPower(0);
					motor_r.SetPower(0);
					motor_l.SetClockwise(sign);
					motor_r.SetClockwise(sign);
				}
				else {
					motor_l.SetPower(0);
					motor_r.SetPower(0);
					motor_l.SetClockwise(sign);
					motor_r.SetClockwise(sign);
				}











				speed_r = (aabbss(speed_r) + 40.0f);
				speed_l = (aabbss(speed_l) + 41.38f);



				if(speed_l > 500 && sign == 0){
					speed_l = 500;
				}
				else if(speed_l < -500 && sign ==1){
					speed_l = -500;
				}


				if(speed_r > 500 && sign ==0){
					speed_r = 500;
				}
				else if(speed_r < -500 && sign ==1){
					speed_r = -500;
				}

				motor_l.SetPower((int16_t)(speed_l));
				motor_r.SetPower((int16_t)(speed_r));







			}





			//			if((int16_t)(t-pt4) >= 1 && yo == 4){
			//				pt5 = System::Time(); //?
			//				yo =0;
			//				encoder_r.Update();
			//				encoder_l.Update();
			//
			//				count_r = (int16_t)(-encoder_r.GetCount());
			//				count_l = (int16_t)(encoder_l.GetCount());
			//
			//				total_count_l += (float)count_l * 0.001;
			//				total_count_r += (float)count_r * 0.001;
			//
			//				last_ir_encoder_error = ir_encoder_error;
			//				ir_encoder_error = balance_count - count_r;
			//				last_il_encoder_error = il_encoder_error;
			//				il_encoder_error = balance_count - count_l;
			//
			//				//				if(turn[0] >= 0.8 && turn [1] >= 0.8)
			//				//					speed = (count_l + count_r)/ 2;
			//				//				else if(turn[0] == 1)
			//				//					speed = count_l;
			//				//				else if(turn[1] == 1)
			//				//					speed = count_r;
			//
			//				//					last_speed_error = speed_error;
			//				//					speed_error = ideal_speed - speed;
			//
			//				ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
			//				il_encoder_errorsum += (float)il_encoder_error * 0.001;
			//
			//				printf("%d, %d, %f, %f, %f\n",mid_point, white_count, ideal_speed, turn_Kp, turn_Kd);
			//
			//			}

			//			if((int16_t)(t - pt7) >= 1000)
			//			{
			//				pt7 = t;
			//
			//				// Change print16_t mode while running
			//				if (button0.IsDown())
			//				{
			//					led0.Switch();
			//					print16_t_data = !print16_t_data;
			//				}
			//
			//
			//				// Enable print16_t mode without running
			//				else if(button1.IsDown())
			//				{
			//					led1.Switch();
			//					float average = 0;
			//					while(1)
			//					{
			//						t = System::Time();
			//
			//						if((int16_t)(t - pt) >= 6)
			//						{
			//							pt = t;
			//							ccd_counter++;
			//
			//							ccd.StartSample();
			//							while (!ccd.SampleProcess()){}
			//							Data = ccd.GetData();
			//
			//							uint16_t sum = 0;
			//
			//							for(int16_t i = 0; i < Tsl1401cl::kSensorW; i++){
			//								Data[i] = Data[i] * 4;
			//								sum += Data[i];
			//							}
			//
			//							average = sum / Tsl1401cl::kSensorW;
			//							if(average > 72)
			//								average = 72;
			//							else if(average < 35)
			//								average = 35;
			//							else
			//								average = average + 3;
			//
			//						}
			//
			//						if (ccd_counter >= 15){
			//							ccd_counter = 0;
			//							St7735r::Rect rect_1, rect_2, rect_3, rect_4;
			//							for(int16_t i = 0; i<Tsl1401cl::kSensorW; i++){
			//								rect_1.x = i;
			//								rect_1.y = 0;
			//								rect_1.w = 1;
			//								rect_1.h = Data[i];
			//								rect_2.x = i;
			//								rect_2.y = Data[i];
			//								rect_2.w = 1;
			//								rect_2.h = 80 - Data[i];
			//								lcd.SetRegion(rect_1);
			//								lcd.FillColor(~0);
			//								lcd.SetRegion(rect_2);
			//								lcd.FillColor(0);
			//							}
			//							for(int16_t i=0; i<Tsl1401cl::kSensorW; i++){
			//								if(Data[i] < average)
			//									Data[i] = 0;
			//								else
			//									Data[i] = 60;
			//							}
			//
			//							for(int16_t i = 0; i<Tsl1401cl::kSensorW; i++){
			//								rect_3.x = i;
			//								rect_3.y = 90;
			//								rect_3.w = 1;
			//								rect_3.h = Data[i];
			//								rect_4.x = i;
			//								rect_4.y = 90 + Data[i];
			//								rect_4.w = 1;
			//								rect_4.h = 60 - Data[i];
			//								lcd.SetRegion(rect_3);
			//								lcd.FillColor(~0);
			//								lcd.SetRegion(rect_4);
			//								lcd.FillColor(0);
			//							}
			//						}
			//
			//						// Break the mode that print16_t pixels without running
			//						if((int16_t)(t - pt8) >= 1000){
			//							pt8 = t;
			//							if(button1.IsDown())
			//							{
			//								led1.Switch();
			//								break;
			//							}
			//						}
			//
			//
			//					}
			//				}
			//			}



		}

	}


}


//	motor_l.SetPower(200);
//	motor_r.SetPower(200);
//	motor_l.SetClockwise(0);
//	motor_r.SetClockwise(0);
//
//
//	while(1){
//
//		encoder_l.Update();
//		encoder_r.Update();
//		System::DelayMs(50);
//		int16_t count_l =  - encoder_l.GetCount();
//		int16_t count_r = encoder_r.GetCount();
//
//
//	}





