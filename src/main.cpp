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
#include <libsc/simple_buzzer.h>

//#include "VarManager.h"

#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define LEFT            0
#define RIGHT           1

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


float ic_Kp = 1500.0f;                   // Recommend 50
float ic_Kd = 5500.0f;            	    // Recommend 45
float ic_Ki = 50;

float is_Kp = 0.0;                  // Recommend 0.05
float is_Ki = 0;
float is_Kd = 0.00;
float speed_Kp = 0;
float speed_Kd = 0;
float ideal_speed = 100;
float speed_PID = 0;

float turn_Kp = 0;         // Recommend
//float turn_Ki = 0;
float turn_Kd = 0;

float encoder_r_Kp = 2.0f;        // Recommend 2
//float encoder_r_Ki = 0.0;   // Recommend 0.0008
//float encoder_r_Kd = 0.0;
float encoder_l_Kp = 3.5f;        // Recommend 2
//float encoder_l_Ki = 0.00;   // Recommend 0.0008
//float encoder_l_Kd = 0.0;

float original_angle = 0;
float anti_friction_angle = 0;
float new_original_angle = 0;
float turn = 0.0f;
float still_Ki = 0.000;
float ratio_old = 0;
float power_l = 0.0f;
float power_r = 0.0f;
float turn_error = 0.0f;
float raw_angle = 0.0f;
float trust_accel = 0.005;

float speed_error_sum = 0;
double _R[2] = {0.01, -1};
Kalman speedKF(0.0001, _R, 0, 1);

int32_t count_l =0;
int32_t count_r =0;

int mid_point = 0;
uint16_t ccd_average = 0;
int border_r = Tsl1401cl::kSensorW - 11, border_l = 10;

float ideal_count = 0;
std::array<uint16_t, Tsl1401cl::kSensorW> Data;
std::array<uint16_t, Tsl1401cl::kSensorW> F_Data;
std::array<uint16_t, Tsl1401cl::kSensorW> A_Data;

// Turning variable
Byte up_ccd = 0;
int low_ideal_speed = 100;
int high_ideal_speed = 100;
int up_mid_point = 63;
int up_l_standard_border = 0;		// undefined
int up_r_standard_border = 0;		// undefined
int down_l_standard_border = 0;		// undefined
int down_r_standard_border = 0;		// undefined
int up_border_l = 0;
int last_up_border_l = 0;
int up_border_r = 0;
int last_up_border_r = 0;
int up_ccd_average = 0;
int up_white_count = 0;
int up_white_count_up_limit = 0;
int up_white_count_low_limit = 0;
int down_white_count_up_limit = 0;
int down_white_count_low_limit = 0;
int min_line_wide = 0;
Byte mid_cross_right = 0;		// Decided by up_border - last_up_border
Byte mid_cross = 0;             // Decided by white_count
Byte cross = 0;
Byte middle_line = 0;
Byte right_angle = 0;
Byte obstacle = 0;              // Decided by up_border - last_up_border
Byte curve = 0;
Byte wave_curve = 0;
Byte black_line = 0;
Byte turn_direction = 0;

int print_ccd = 0;
Byte decided_angle = 0;
void SelectLeft(const uint8_t id)
{
	print_ccd = 0;
}

void SelectRight(const uint8_t id)
{
	print_ccd = 1;
}

void SelectUp(const uint8_t id)
{
	decided_angle = !decided_angle;
}

void SelectDown(const uint8_t id)
{
}

void Select(const uint8_t id)
{
	up_l_standard_border = up_border_l;
	up_r_standard_border = up_border_r;
}

int Middle_pass_filter(int pixel);
void Pixel_filter(std::array<uint16_t, Tsl1401cl::kSensorW> pixel);

int main()
{
	std::array<float, 3>accel;
	std::array<float, 3>omega;
	std::array<float, 3>raw_accel;

	//intialize the system
	System::Init();
	Timer::TimerInt t = 0;
	Timer::TimerInt pt = t;
	pt = System::Time();

	RemoteVarManager* varmanager = new RemoteVarManager(4);

	//	Initalize the BT module
	JyMcuBt106::Config bt_config;
	bt_config.id = 0;
	bt_config.rx_irq_threshold = 2;
	bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	bt_config.rx_isr = std::bind(&RemoteVarManager::OnUartReceiveChar, varmanager, std::placeholders::_1);
	JyMcuBt106 bt(bt_config);




	libutil::InitDefaultFwriteHandler(&bt);

	RemoteVarManager::Var* turn_Kp = varmanager->Register("turn_Kp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* turn_Kd = varmanager->Register("turn_Kd",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* is_Kp = varmanager->Register("is_Kp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* is_Kd = varmanager->Register("is_Kd",RemoteVarManager::Var::Type::kReal);

	printf("turn_Kp,real,0,0\n");
	printf("turn_Kd,real,1,0\n");
	printf("is_Kp,real,2,0\n");
	printf("is_Kd,real,3,0\n");



	Mpu6050::Config gyro_config;
	//sensitivity of gyro
	gyro_config.gyro_range = Mpu6050::Config::Range::kLarge;
	//sensitivity of accelerometer
	gyro_config.accel_range = Mpu6050::Config::Range::kLarge;
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
	R[0] = 0.005;
	Kalman mid_point_KF(0.00001,R,0,1);

	Joystick::Config joyconfig;
	joyconfig.id = 0;
	joyconfig.is_active_low = true;
	joyconfig.listener_triggers[(uint8_t)Joystick::State::kUp] = Joystick::Config::Trigger::kDown;
	joyconfig.listeners[(uint8_t)Joystick::State::kUp] =  &SelectUp;
	joyconfig.listener_triggers[(uint8_t)Joystick::State::kDown] = Joystick::Config::Trigger::kDown;
	joyconfig.listeners[(uint8_t)Joystick::State::kDown] =  &SelectDown;
	joyconfig.listener_triggers[(uint8_t)Joystick::State::kLeft] = Joystick::Config::Trigger::kDown;
	joyconfig.listeners[(uint8_t)Joystick::State::kLeft] =  &SelectLeft;
	joyconfig.listener_triggers[(uint8_t)Joystick::State::kRight] = Joystick::Config::Trigger::kDown;
	joyconfig.listeners[(uint8_t)Joystick::State::kRight] =  &SelectRight;
	joyconfig.listener_triggers[(uint8_t)Joystick::State::kSelect] = Joystick::Config::Trigger::kDown;
	joyconfig.listeners[(uint8_t)Joystick::State::kSelect] =  &Select;
	Joystick joy(joyconfig);

	DirMotor::Config l_motor;
	l_motor.id = 0;
	DirMotor motor_l(l_motor);

	DirMotor::Config r_motor;
	r_motor.id = 1;
	DirMotor motor_r(r_motor);

	AbEncoder::Config enconfig;
	enconfig.id = 0;
	AbEncoder encoder_l(enconfig);

	AbEncoder::Config r_encoder;
	r_encoder.id = 1;
	AbEncoder encoder_r(r_encoder);

	Tsl1401cl ccd_down(0);
	Tsl1401cl ccd_up(1);

	St7735r::Config config1;
	config1.is_revert = false;
	St7735r lcd(config1);

//    Led::Config led_config0;
//    led_config0.id = 0;
//    Led led0(led_config0);
//
//    Led::Config led_config1;
//    led_config1.id = 1;
//    Led led1(led_config1);

	libbase::k60::Adc::Config tuner_config;
	tuner_config.pin = libbase::k60::Pin::Name::kPtd1;
	libbase::k60::Adc angle_tuner(tuner_config);

	SimpleBuzzer::Config buzzer_config;
	buzzer_config.id = 0;
	buzzer_config.is_active_low = 1;
	SimpleBuzzer buzzer(buzzer_config);

	float angle_offset;

	System::DelayMs(25);

	motor_l.SetPower(0);
	motor_r.SetPower(0);
	count_l = 0;
	count_r = 0;

	float speed = 0;

	t= System::Time();
	pt = t;

	motor_l.SetPower(0);
	motor_r.SetPower(0);
	while(1){

		mma8451q.Update();
		System::DelayMs(4);
		accel = mma8451q.GetAccel();
		raw_angle = accel[1]*57.29578 + 0.7f;
		t = System::Time();
		if((t-pt)>=2000)
			break;
	}

	original_angle = raw_angle;

	float accel_angle = original_angle;
	float raw_accel_angle = 0;
	float last_gyro_angle = original_angle;
	float gyro_angle = original_angle;
	float last_accel_angle = original_angle;
	float output_angle = 0;            //karmen filtered
	float gyro_offset = 0;

	float total_count_l =0;
	float total_count_r =0;
	float last_angle_error = 0;
	float now_angle_error = 0;
	float angle_error_change = 0;
	float angle_error_sum = 0.0f;


    float last_speed_error = 0;
	float speed_error = 0;
    float speed_error_change = 0;

    int count = 0;

	int32_t last_il_encoder_error = 0;
	int32_t last_ir_encoder_error = 0;
	int32_t ir_encoder_error = 0;
	int32_t il_encoder_error = 0;
	int32_t ir_encoder_error_change = 0;
	int32_t il_encoder_error_change = 0;
	int32_t last_ir_encoder_error_change = 0;
	int32_t last_il_encoder_error_change = 0;
	float ir_encoder_errorsum = 0.0f;
	float il_encoder_errorsum = 0.0f;
	double kalman_value[2] = {0.01, -1};
	Kalman acc(0.0001f, kalman_value, 0, 1);
	kalman_value[0] = 0.0005;
	Kalman gyro(0.0001f, kalman_value, 0, 1);
	//	KF m_gyro_kf[3];
	//	float kalman_value[2] = {0.3f, 1.5f};
	//	kalman_filter_init(&m_gyro_kf[0], 0.01f, &kalman_value, original_angle, 1);
	float speed_l = 0;                    //last output to motor left,0-1000
	float speed_r = 0;                    //last output to motor right,0-1000

	uint32_t pt0 = 0;
	uint32_t pt1 = 0;
	uint32_t pt2 = 0;
	uint32_t pt3 = 0;
	uint32_t pt4 = 0;
	uint32_t pt5 = 0;
	uint32_t pt6 = 0;
	uint32_t pt7 = 0;
	uint32_t pt8 = 0;
	uint32_t right_angle_time = 0;


	int sign = 0;
	int last_ideal_count = 0;
	int ccd_counter = 0;
	bool print_data = false;
	int white_count = 0;
	int black_count = 0;
	int turn_count = 0;

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
//
//			if((int32_t)(t-pt8) >= 1000)
//			{
//				pt8 = t;
//				switch(int(ideal_count))
//				{
//				case 10:
//					ideal_count = 200.0f;
//					break;
//				case 200:
//					ideal_count = 300.0f;
//					break;
//				case 300:
//					ideal_count = -10.0f;
//					break;
//				case -10:
//					ideal_count = -200.0f;
//					break;
//				case -200:
//					ideal_count = -300.0f;
//					break;
//				case -300:
//					ideal_count = 10.0f;
//					break;
//
//				default:
//					ideal_count = 10.0f;
//					break;
//				}
//			}

			if((int32_t)(t-pt1) >= 7  && yo==0){
				pt1 = System::Time();
				yo = 1;

				encoder_r.Update();
				encoder_l.Update();

				count += (int32_t)(encoder_l.GetCount());

				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count - count_r;
				last_il_encoder_error = il_encoder_error;
				il_encoder_error = ideal_count - count_l;

				speed = (count_l + count_r)/ 2;

				last_speed_error = speed_error;
				speed_error = ideal_speed - speed;

				ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
				il_encoder_errorsum += (float)il_encoder_error * 0.001;


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
				accel_angle = accel[1]*57.29578;
				//accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;

				//last_gyro_angle = gyro_angle;
				double __temp = 0;
				gyro.Filtering(&__temp, omega[1],0);
				omega[1] = __temp;
				gyro_angle += omega[1] * 0.0035;
				gyro_offset += trust_accel * (accel_angle - output_angle);

				output_angle = gyro_angle + gyro_offset;
				last_angle_error = now_angle_error;
				now_angle_error = tan((original_angle - output_angle) / 57.29578);
				angle_error_change = now_angle_error -last_angle_error;
				angle_error_sum += now_angle_error;

				ideal_count = ic_Kp * now_angle_error  + ic_Kd * angle_error_change + ic_Ki * angle_error_sum;
			}



			if((int32_t)(t-pt1) >= 1 && yo ==1){
				//				lincoln.Turn();
				pt2=System::Time();

				//				pt1 = t;
				yo = 2;

				encoder_r.Update();
				encoder_l.Update();

				count += (int32_t)(encoder_l.GetCount());

				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;

				speed = (count_l + count_r)/ 2;
				double _temp = 0;
				speedKF.Filtering(&_temp, speed, 0);
				speed = _temp;
			    speed_error_change = (speed_error - last_speed_error);  // Delayed error change
//				last_speed_error = speed_error;
				speed_error = ideal_speed - speed;
				speed_error_sum += speed_error * 0.0035;

			    if(ideal_speed != 0)
			    	anti_friction_angle = ideal_speed * 0.0017f;
			    original_angle = raw_angle - anti_friction_angle -
			    		libutil::Clamp<float>(-5.0f, is_Kp->GetReal() * (0.8f * speed_error + 0.2f * last_speed_error) + is_Kd->GetReal() * speed_error_change + is_Ki * speed_error_sum, 5.0f);
			    last_speed_error = 0.8f * speed_error + 0.2f * last_speed_error;
			    anti_friction_angle = 0;

			    last_ir_encoder_error = ir_encoder_error;
			    ir_encoder_error = ideal_count - count_r;
			    last_il_encoder_error = il_encoder_error;
			    il_encoder_error = ideal_count - count_l;

			    ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
			    il_encoder_errorsum += (float)il_encoder_error * 0.001;

			    speed_PID = ideal_speed + speed_Kp * speed_error + speed_Kd * speed_error_change;

			    if(speed_Kp == 0)
			    	speed_PID = 0;

				power_r = ideal_count - speed_PID + (float)ir_encoder_error * encoder_r_Kp;
				power_l = ideal_count - speed_PID + (float)il_encoder_error * encoder_l_Kp;

			    //					if(turn > 0)
			    //						power_r += turn;
			    //					else
			    //						power_l -= turn;

			    speed_r = 1.935 *(power_r + turn);
			    speed_l = 1.745 *(power_l - turn);

			    if(speed_l >= 0)
			    	sign = 0;
			    else if(speed_l < 0)
			    	sign = 1;

			    motor_l.SetClockwise(sign);
			    motor_r.SetClockwise(!sign);

			    if(speed_l > 900)
			    	speed_l = 900;
			    else if(speed_l < -900)
			    	speed_l = -900;

			    if(speed_r > 900)
			    	speed_r = 900;
			    else if(speed_r < -900)
			    	speed_r = -900;

			    if(right_angle == 11)
			    {
			    	if(turn_direction == RIGHT)
			    	{
			    		motor_r.SetPower(0);
			    		motor_l.SetPower(int(abs(1.745 * count_l) + 25.3f));
			    	}
			    	else if(turn_direction == LEFT)
			    	{
			    		motor_r.SetPower(int(abs(1.935 * count_r) + 27.6f));
			    		motor_l.SetPower(0);
			    	}
			    }

			    else
			    {
			    	motor_l.SetPower(int(abs(speed_l) + 25.3f));
			    	motor_r.SetPower(int(abs(speed_r) + 27.6f));
			    }
			}



			if((int32_t)(t-pt2) >= 1 && yo == 2){
				pt0 = System::Time();
				yo = 8;

				buzzer.SetBeep(0);

				encoder_r.Update();
				encoder_l.Update();
				count += (int32_t)(encoder_l.GetCount());

				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count - count_r;
				last_il_encoder_error = il_encoder_error;
				il_encoder_error = ideal_count - count_l;

				speed = (count_l + count_r)/ 2;

				last_speed_error = speed_error;
				speed_error = ideal_speed - speed;

				ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
				il_encoder_errorsum += (float)il_encoder_error * 0.001;

				// For the top CCD to prediction
				if(up_ccd == 1)
				{
					up_ccd = 0;
					while(! ccd_up.SampleProcess()){};
					A_Data = ccd_up.GetData();  // 0 - 127 is left to right from the view of CCD
					ccd_up.StartSample();
					uint32_t ccd_sum = 0;

					for(int i = 15; i < Tsl1401cl::kSensorW - 15; i++){
						ccd_sum += A_Data[i];
					}
					up_ccd_average = ccd_sum / (Tsl1401cl::kSensorW - 30);    // print

					if(up_ccd_average > 160)
						up_ccd_average = up_ccd_average - 10;
					else if(up_ccd_average < 140)
						up_ccd_average = up_ccd_average + 10;

					up_white_count = 0;
					for(int i = 15; i < Tsl1401cl::kSensorW - 15; i++){
						if(A_Data[i] < up_ccd_average)
							A_Data[i] = 0;
						else
						{
							A_Data[i] = 1;
							up_white_count++;
						}
					}

					if(black_line == 0 && middle_line == 0 && cross == 00 && up_white_count >= 96)
					{
						cross = 10;
						right_angle = 00;
						buzzer.SetBeep(1);
					}

					if(cross == 10 && up_white_count <= 82)
						cross = 00;

					for(int i = up_mid_point; i >= 15; i--)
					{
						up_border_l = i;
						if(A_Data[i] == 0)
							break;
					}

					for(int i = up_mid_point; i < Tsl1401cl::kSensorW - 15; i++)
					{
						up_border_r = i;
						if(A_Data[i] == 0)
							break;
					}

					// Detect the right angle
					if(black_line == 0 &&
						middle_line == 0 &&
						cross == 00 &&
						right_angle == 00)
					{
						if(abs(up_border_l - up_l_standard_border) <= 3 && up_border_r >= 112)
						{
							right_angle = 10;
							turn_direction = RIGHT;
							buzzer.SetBeep(1);
							// Change other states to 0
						}

						else if(abs(up_border_r - up_r_standard_border) <= 3 && up_border_l <= 16)
						{
							right_angle = 10;
							turn_direction = LEFT;
							// Change other states to 0
						}
					}

					else if(right_angle == 10)
					{
						if(up_white_count <= 40)
							right_angle == 11;
					}

					else if(right_angle == 11)
					{
						if(turn_direction == RIGHT)
						{
							if(abs(up_border_l - up_l_standard_border) <= 3)
								right_angle = 00;
						}

						else if(turn_direction == LEFT)
						{
							if(abs(up_border_r - up_r_standard_border) <= 3)
								right_angle == 00;
						}
					}

					// right_angle self-correcting
					if(right_angle == 10)
					{
						if(abs(up_border_r - up_r_standard_border) <= 3 && abs(up_border_l - up_l_standard_border) <= 3)
							right_angle = 00;
					}


					up_mid_point = (up_border_l + up_border_r) / 2;  // print
				}

				/*  The below CCD's turn.
				 *  It decides the middle line.
				 *  And cancels states.
				 *
				 *
				 */
				else
				{
					up_ccd = 1;
					while(! ccd_down.SampleProcess()){};
					Data = ccd_down.GetData();  // 0 - 127 is left to right from the view of CCD
					ccd_down.StartSample();
					uint32_t ccd_sum = 0;

					for(int i = 7; i < Tsl1401cl::kSensorW - 7; i++){
						F_Data[i] = Data[i];
						ccd_sum += Data[i];
					}
					Pixel_filter(F_Data);
					ccd_average = ccd_sum / (Tsl1401cl::kSensorW - 14);    // print

					if(ccd_average > 160)
						ccd_average = ccd_average - 10;
					else if(ccd_average < 140)
						ccd_average = ccd_average + 10;

					white_count = 0;
					black_count = 0;
					for(int i = 7; i < Tsl1401cl::kSensorW - 7; i++){
						if(Data[i] < ccd_average)
						{
							A_Data[i] = 0;
							if(i >= 54 && i <= 72)
								black_count++;
						}
						else
						{
							A_Data[i] = 1;
							white_count++;
						}
					}

					if(white_count <= 30 && black_line == 0 && right_angle == 0)
					{
						black_line = 1;
						buzzer.SetBeep(1);
						middle_line = 0;
						if(ideal_speed == low_ideal_speed)
							ideal_speed = high_ideal_speed;
						else
							ideal_speed = low_ideal_speed;
					}

					else if(white_count >= 80 && black_line == 1)
						black_line = 0;

					if(black_count >= 4 && middle_line == 0 && black_line == 0 && right_angle == 0)   // added right_angle
					{
						middle_line = 1;
						buzzer.SetBeep(1);
						// Change other states to 0
						cross = 00;
					}

					if(middle_line == 1)
					{
						for(int i = 20; i <= 100; i++)
						{
							border_l = i;
							if(A_Data[i] == 0)
								break;
						}

						for(int i = 100; i >= 20; i--)
						{
							border_r = i;
							if(A_Data[i] == 0)
								break;
						}

						if(border_l > border_r)
						{
							middle_line = 0;
							border_l = mid_point;
							border_r = mid_point;
						}
					}

					else if(right_angle == 10 && white_count <= 60)
						right_angle = 11;

					else{
						for(int i = mid_point; i >= 7; i--)
						{
							border_l = i;
							if(A_Data[i] == 0)
								break;
						}

						for(int i = mid_point; i < Tsl1401cl::kSensorW - 7; i++)
						{
							border_r = i;
							if(A_Data[i] == 0)
								break;
						}
					}

						mid_point = (border_l + border_r) / 2;  // print
				}

//				double temp_ = 0;
//				mid_point_KF.Filtering(&temp_, mid_point,0);
//				mid_point = temp_;

//				int a = 63, b = 64;
//
//				while(! ccd.SampleProcess()){};
//				Data = ccd.GetData();  // 0 - 127 is left to right from the view of CCD
//				ccd.StartSample();
//				uint32_t ccd_sum = 0;
//
//				for(int i = 10; i < Tsl1401cl::kSensorW - 10; i++){
//					Data[i] = (float)Data[i] * 80.0f * 3.8f / 256.0f;
//					ccd_sum += Data[i];
//				}
//
//				St7735r::Rect rect_2;
//				for(int i = 0; i<Tsl1401cl::kSensorW; i++){
//					rect_2.x = i;
//					rect_2.y = Data[i];
//					rect_2.w = 1;
//					rect_2.h = 1;
//					lcd.SetRegion(rect_2);
//					lcd.FillColor(~0);
//				}
//
//				ccd_average = ccd_sum / (Tsl1401cl::kSensorW - 20);
//
//				if(ccd_average > 70)
//					ccd_average = 72;
//				else if(ccd_average < 35)
//					ccd_average = 35;
//				else
//					ccd_average = ccd_average + 3;
//
//				white_count = 0;
//
//				for(int i = 10; i < Tsl1401cl::kSensorW - 10; i++){
//					if(Data[i] < ccd_average)
//						Data[i] = 0;
//					else
//					{
//						Data[i] = 1;
//						white_count++;
//					}
//				}
//
//				// left is 0
//				if(Data[63] == 1){
//					for (a = a + 1; a < Tsl1401cl::kSensorW - 10; a++)
//					{
//						if(Data[a] == 1)
//							border_r = a;
//						else
//							break;
//					}
//
//					for(b = b - 1; b >= 10; b--)
//					{
//						if(Data[b] == 1)
//							border_l = b;
//						else
//							break;
//					}
//
//					//						while(border_r - border_l < 80)
//					//						{
//					//							for (a = a + 1; a < Tsl1401cl::kSensorW; a++)
//					//							{
//					//								if(Data[a] == 60)
//					//									border_r = a;
//					//								else
//					//									break;
//					//							}
//					//
//					//							for(b = b - 1; b >= 0; b--)
//					//							{
//					//								if(Data[b] == 60)
//					//									border_l = b;
//					//								else
//					//									break;
//					//							}
//					//						}
//				}
//
//				else
//				{
//					if(mid_point >= 63)
//					{
//						for(int i = 64; i < Tsl1401cl::kSensorW - 10; i++)
//						{
//							if (Data[i] == 1)
//							{
//								border_l = i;
//								border_r = Tsl1401cl::kSensorW - 11;
//								break;
//							}
//							else{
//								border_l = i;
//								border_r = i;
//							}
//						}
//					}
//
//					else
//					{
//						for(int i = 62; i >= 10; i--)
//						{
//							if (Data[i] == 1)
//							{
//								border_r = i;
//								border_l = 10;
//								break;
//							}
//							else{
//								border_l = i;
//								border_r = i;
//							}
//						}
//					}
//				}
//
//				mid_point = (border_l + border_r) / 2;
				float last_turn_error = turn_error;
				if(black_line == 1 || cross == 11)
					turn_error = 63 - up_mid_point;
				else
					turn_error = 63 - mid_point;
				float turn_error_change = turn_error - last_turn_error;
				float hehe = turn_Kp->GetReal();

				turn = 0.0f;
				turn_count++;
				if(turn_count >= 2)
				{
					turn_count = 0;
					turn = hehe * turn_error + turn_error_change * turn_Kd->GetReal();
				}
			}



			/*second round to get angle and encoder
			 *
			 *
			 *
			 *
			 */
			if((int32_t)(t-pt0) >= 2  && yo == 8){
				pt3 = System::Time();

				yo = 3;

				encoder_r.Update();
				encoder_l.Update();

				count += (int32_t)(encoder_l.GetCount());
				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count - count_r;
				last_il_encoder_error = il_encoder_error;
				il_encoder_error = ideal_count - count_l;
				il_encoder_error_change = il_encoder_error - last_il_encoder_error;
				ir_encoder_error_change = ir_encoder_error - last_ir_encoder_error;

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;

				speed = (count_l + count_r)/ 2;

				last_speed_error = speed_error;
				speed_error = ideal_speed - speed;

				ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
				il_encoder_errorsum += (float)il_encoder_error * 0.001;

				mpu6050.Update();
				mma8451q.Update();

				accel = mma8451q.GetAccel();
				omega = mpu6050.GetOmega();

				raw_accel = mma8451q.GetAccel();
				raw_accel_angle = raw_accel[1] * 57.29578;
				last_accel_angle = accel_angle;
				double temp = 0;
				acc.Filtering(&temp, (double)accel[1], 0);
				accel[1] = temp;
				accel_angle = accel[1]*57.29578;

				double __temp = 0;
				gyro.Filtering(&__temp, omega[1],0);
				omega[1] = __temp;
				gyro_angle += omega[1] * 0.0035;
				gyro_offset += trust_accel * (accel_angle - output_angle);

				output_angle = gyro_angle + gyro_offset;
				last_angle_error = now_angle_error;
				now_angle_error = tan((original_angle - output_angle) / 57.29578);
				angle_error_change = now_angle_error -last_angle_error;
				angle_error_sum += now_angle_error;

				ideal_count = ic_Kp * now_angle_error  + ic_Kd * angle_error_change + ic_Ki * angle_error_sum;
			}


			if((int32_t)(t-pt3) >= 1 && yo == 3){
				//				lincoln.Turn();
				pt4 = System::Time();

				//				pt1 = t;
				yo = 4;

				encoder_r.Update();
				encoder_l.Update();
				count += (int32_t)(encoder_l.GetCount());

				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;

				speed = (count_l + count_r)/ 2;
				double _temp = 0;
				speedKF.Filtering(&_temp, speed, 0);
				speed = _temp;

			    speed_error_change = (speed_error - last_speed_error);  // Delayed error change
//				last_speed_error = speed_error;
				speed_error = ideal_speed - speed;

			    if(ideal_speed != 0)
			    	anti_friction_angle = ideal_speed * 0.0017f;
			    original_angle = raw_angle - anti_friction_angle -
			    		libutil::Clamp<float>(-5.0f, is_Kp->GetReal() * (0.8f * speed_error + 0.2f * last_speed_error) + is_Kd->GetReal() * speed_error_change + is_Ki * speed_error_sum, 5.0f);

			    anti_friction_angle = 0;

			    last_ir_encoder_error = ir_encoder_error;
			    ir_encoder_error = ideal_count - count_r;
			    last_il_encoder_error = il_encoder_error;
			    il_encoder_error = ideal_count - count_l;
			    il_encoder_error_change = il_encoder_error - last_il_encoder_error;
			    ir_encoder_error_change = ir_encoder_error - last_ir_encoder_error;

			    ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
			    il_encoder_errorsum += (float)il_encoder_error * 0.001;

			    speed_PID = ideal_speed + speed_Kp * speed_error + speed_Kd * speed_error_change;

			    if(speed_Kp == 0)
			    	speed_PID = 0;

				power_r = ideal_count - speed_PID + (float)ir_encoder_error * encoder_r_Kp;
				power_l = ideal_count - speed_PID + (float)il_encoder_error * encoder_l_Kp;

//					if(turn > 0)
//						power_r += turn;
//					else
//						power_l -= turn;

			    speed_r = 1.935 *(power_r + turn);
			    speed_l = 1.745 *(power_l - turn);

			    if(speed_l >= 0)
			    	sign = 0;
			    else if(speed_l < 0)
			    	sign = 1;

			    motor_l.SetClockwise(sign);
			    motor_r.SetClockwise(!sign);

			    if(speed_l > 900)
			    	speed_l = 900;
			    else if(speed_l < -900)
			    	speed_l = -900;

			    if(speed_r > 900)
			    	speed_r = 900;
			    else if(speed_r < -900)
			    	speed_r = -900;

			    if(right_angle == 11)
			    {
			    	if(turn_direction == RIGHT)
			    	{
			    		motor_r.SetPower(0);
			    		motor_l.SetPower(int(abs(1.745 * count_l) + 25.3f));
			    	}
			    	else if(turn_direction == LEFT)
			    	{
			    		motor_r.SetPower(int(abs(1.935 * count_r) + 27.6f));
			    		motor_l.SetPower(0);
			    	}
			    }

			    else
			    {
			    	motor_l.SetPower(int(abs(speed_l) + 25.3f));
			    	motor_r.SetPower(int(abs(speed_r) + 27.6f));
			    }
			}


			if((int32_t)(t-pt4) >= 1 && yo == 4){
				pt5 = System::Time(); //?
				yo =0;
				encoder_r.Update();
				encoder_l.Update();
				count += (int32_t)(encoder_l.GetCount());

				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count - count_r;
				last_il_encoder_error = il_encoder_error;
				il_encoder_error = ideal_count - count_l;

				speed = (count_l + count_r)/ 2;

				last_speed_error = speed_error;
				speed_error = ideal_speed - speed;

				ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
				il_encoder_errorsum += (float)il_encoder_error * 0.001;

				if(angle_error_sum >= 5)
					angle_error_sum = 5;
				else if(angle_error_sum <= -5)
					angle_error_sum = -5;

//				angle_offset = float(angle_tuner.GetResult()) / 32.0f - 4.0f;
//				original_angle += angle_offset;


//				if(print_ccd == 0)
//				{
//					Byte buf[128 + 2];
//					buf[0] = 'L';
//					for(int i=0; i<128; i++){
//						buf[i + 1] = Data[i];
//					}
//					buf[sizeof(buf) - 1] = '\n';
//					bt.SendBuffer(buf, sizeof(buf));
//				}
//
//				else
//				{
//					Byte buf[128 + 2];
//					buf[0] = 'L';
//					for(int i=0; i<128; i++){
//						buf[i + 1] = F_Data[i];
//					}
//					buf[sizeof(buf) - 1] = '\n';
//					bt.SendBuffer(buf, sizeof(buf));
//				}

				printf("%f, %f, %f, %f\n", original_angle, output_angle, ideal_count, speed);
//				if(print_ccd == 0)
//				printf("%d, %d, %d, %f, %d, %d\n", mid_point, border_l, border_r, original_angle, cross, middle_line);
//
//				else
//				printf("%d, %d, %d, %d, %d, %d\n", up_mid_point, up_border_l, up_border_r, black_line, right_angle, middle_line);
			}
		}
	}
}


int Middle_pass_filter(uint16_t pixel)
{
	static uint16_t array_flag = 0;
	static uint16_t past_data[128] = {};
	static uint16_t new_data[128] = {};
	static uint16_t temp = 0;

	for(int i = 0;i < 3 - 1;i++){
		past_data[i] = past_data[i+1];
	}
	past_data[3-1] = pixel;
for(int i = 0;i < 3;i++){
        new_data[i] = past_data[i];
}
//cout << "Your past three numbers is:" << new_data[0] << " "<< new_data[1] << " " << new_data[2] << "\n";
	if(++array_flag < 3){

	}
	else{
		for(int i = 0;i < 3 - 1;i++){
			for(int a = i + 1;a < 3;a++){
				if(new_data[i] <= new_data[a]){

					continue;
				}
				else{
					temp = new_data[i];
					new_data[i] = new_data[a];
					new_data[a] = temp;
				}
				if(new_data[0] < new_data[1] && new_data[1] < new_data[2]){
					break;
				}

			}
			if(new_data[0] < new_data[1] && new_data[1] < new_data[2]){
					break;
				}

		}
		pixel = new_data[1];
past_data[1] = pixel;
	}
	return pixel;
}


void Pixel_filter(std::array<uint16_t, Tsl1401cl::kSensorW> pixel)
{
    for(int i = 7;i < 120;i++){
       pixel[i-1] = Middle_pass_filter(pixel[i]);
    }
}
