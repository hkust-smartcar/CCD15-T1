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

//#include "VarManager.h"

#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

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


float ic_Kp = 12;                   // Recommend 30
float ic_Kd = 0.0005;              // Recommend

float is_Kp = 0;                  // Recommend 0.05
float is_Ki = 0;
float is_Kd = 0.000;
float speed_Kp = 0;
float speed_Kd = 0;
float ideal_speed = 150;
float speed_PID = 0;

//float turn_Kp = 0;         // Recommend
//float turn_Ki = 0;
//float turn_Kd = 0;

//float encoder_r_Kp = 1.3;        // Recommend 2
//float encoder_r_Ki = 0.0;   // Recommend 0.0008
//float encoder_r_Kd = 0.0;
//float encoder_l_Kp = 4.6;        // Recommend 2
//float encoder_l_Ki = 0.00;   // Recommend 0.0008
//float encoder_l_Kd = 0.0;

float original_angle = 0;
float anti_friction_angle = 0;
float new_original_angle = 0;
float turn = 0;
float still_Ki = 0.000;
float ratio_old = 0;
float ratio_new = 1-ratio_old;
int32_t first_count = 0;
float trust_accel = 0.01;
float trust_old_accel = 0;
float trust_new_accel = 1- trust_old_accel;
float power_l = 0.0f;
float power_r = 0.0f;
float turn_error = 0;

int32_t count_l =0;
int32_t count_r =0;

int mid_point = 0;
uint16_t ccd_average = 0;

float lincoln1 = 0;

int32_t ideal_count = 0;
//float ideal_speed = 10;
std::array<uint16_t, Tsl1401cl::kSensorW> Data;


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

	RemoteVarManager::Var* encoder_l_Kp = varmanager->Register("encoder_l_Kp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* encoder_r_Kp = varmanager->Register("encoder_r_Kp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* encoder_l_Kd = varmanager->Register("encoder_l_Kd",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* encoder_r_Kd = varmanager->Register("encoder_r_Kd",RemoteVarManager::Var::Type::kReal);

	printf("encoder_l_Kp,real,0,0\n");
	printf("encoder_r_Kp,real,1,0\n");
	printf("encoder_l_Kd,real,2,0\n");
	printf("encoder_r_Kd,real,3,0\n");



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

	double R[2] = {0.01, -1};

	Kalman Kalman_l(0.0001, R, 0, 1);
	Kalman Kalman_r(0.0001, R, 0, 1);

//	Joystick::Config joycon;
//	joycon.id = 0;
//	joycon.is_active_low = true;
//	Joystick joy(joycon);

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

	float speed = 0;

	float raw_angle;
	t= System::Time();
	pt = t;
	while(1){

		mpu6050.Update();
		System::DelayMs(4);
		accel = mpu6050.GetAccel();
		raw_angle = accel[1]*57.29578;

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

	float total_count_l =0;
	float total_count_r =0;
	float last_angle_error = 0;
	float now_angle_error = 0;
	float angle_error_change = 0;

    float last_speed_error = 0;
	float speed_error = 0;
    float speed_error_change = 0;

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
	double kalman_value[2] = {0.1, -1.0};
	Kalman acc(0.001f, kalman_value, 0, 1);
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


	int sign = 0;
	int last_sign = 0;
	int last_ideal_count = 0;
	int ccd_counter = 0;
	bool print_data = false;
	int white_count = 0;

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
//				switch(ideal_count)
//				{
//				case 10:
//					ideal_count = 200;
//					break;
//				case 200:
//					ideal_count = 300;
//					break;
//				case 300:
//					ideal_count = -10;
//					break;
//				case -10:
//					ideal_count = -200;
//					break;
//				case -200:
//					ideal_count = -300;
//					break;
//				case -300:
//					ideal_count = 10;
//					break;
//
//				default:
//					ideal_count = 10;
//					break;
//				}
//			}

			if((int32_t)(t-pt1) >= 7  && yo==0){
				pt1 = System::Time();
				yo = 1;

				encoder_r.Update();
				encoder_l.Update();

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

//				accel = mpu6050.GetAccel();
//				omega = mpu6050.GetOmega();
//
//				raw_accel = mpu6050.GetAccel();
//				raw_accel_angle = raw_accel[0] * 57.29578;
//				last_accel_angle = accel_angle;
//				double temp = 0;
//				acc.Filtering(&temp, (double)accel[0], 0);
//				accel[0] = temp;
//				accel_angle = accel[0]*57.29578;
//				//accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;
//
//				//last_gyro_angle = gyro_angle;
//				gyro_angle += -omega[1] * 0.0035 + trust_accel * (accel_angle - gyro_angle);
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
//				ideal_count = (int32_t)(ic_Kp * now_angle_error  + ic_Kd * angle_error_change - still_Ki * total_count_r);
			}



			if((int32_t)(t-pt1) >= 1 && yo ==1){
				//				lincoln.Turn();
				pt2=System::Time();

				//				pt1 = t;
				yo = 2;

				encoder_r.Update();
				encoder_l.Update();

				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;

				speed = (count_l + count_r)/ 2;

				last_speed_error = speed_error;
				speed_error = ideal_speed - speed;
			    speed_error_change = (speed_error - last_speed_error);

//			    if(ideal_speed->GetReal() != 0)
//			    	anti_friction_angle = 0.5;
			    original_angle = raw_angle + anti_friction_angle - is_Kp * speed_error - is_Kd * speed_error_change;
			    anti_friction_angle = 0;

//			    if(turn[0] < 0.1 || turn[1] < 0.1)
//			    	 original_angle = raw_angle;

					last_ir_encoder_error = ir_encoder_error;
					ir_encoder_error = ideal_count - count_r;
					last_il_encoder_error = il_encoder_error;
					il_encoder_error = ideal_count - count_l;

					ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
					il_encoder_errorsum += (float)il_encoder_error * 0.001;

//					speed_PID = ideal_speed + speed_Kp * speed_error + speed_Kd * speed_error_change;
//
//					if(speed_Kp == 0)
//						speed_PID = 0;
//
//					power_r = (float)ideal_count + speed_PID + (float)ir_encoder_error * encoder_r_Kp->GetReal() + ir_encoder_error_change * encoder_r_Kd->GetReal();
//					power_l = (float)ideal_count + speed_PID + (float)il_encoder_error * encoder_l_Kp->GetReal() + il_encoder_error_change * encoder_l_Kd->GetReal();
//
//					if(turn > 0)
//						power_r += turn;
//					else
//						power_l -= turn;
//
//					speed_r = 1.8978f * power_r;
//					speed_l = 1.6353f * power_l;
//
//					if(speed_l >= 0)
//						sign = 0;
//					else if(speed_l < 0)
//						sign = 1;
//
//					motor_l.SetClockwise(sign);
//					motor_r.SetClockwise(sign);
//
//					if(speed_l > 900)
//						speed_l = 900;
//					else if(speed_l < -900)
//						speed_l = -900;
//
//					if(speed_r > 900)
//						speed_r = 900;
//					else if(speed_r < -900)
//						speed_r = -900;
//
//					motor_l.SetPower(int(abs(speed_l) + 49.50f));
//					motor_r.SetPower(int(abs(speed_r) + 50.20f));
			}



			if((int32_t)(t-pt2) >= 1 && yo == 2){
//				howard.Set(1);
				pt0 = System::Time();
				yo = 8;

				encoder_r.Update();
				encoder_l.Update();

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

//				// Clean old lcd pixels
//				if(print_data){
//					St7735r::Rect rect_1;
//					for(int i = 0; i<Tsl1401cl::kSensorW; i++){
//						rect_1.x = i;
//						rect_1.y = Data[i];
//						rect_1.w = 1;
//						rect_1.h = 1;
//						lcd.SetRegion(rect_1);
//						lcd.FillColor(0);
//					}
//				}
//
//				int a = 63, b = 64;
//				int border_r = Tsl1401cl::kSensorW - 1, border_l = 0;
//
//                while(! ccd.SampleProcess()){};
//				Data = ccd.GetData();  // 0 - 127 is left to right from the view of CCD
//				ccd.StartSample();
//				uint32_t ccd_sum = 0;
//
//				for(int i = 0; i < Tsl1401cl::kSensorW; i++){
//					Data[i] = (float)Data[i] * 80.0f * 3.8f / 256.0f;
//					ccd_sum += Data[i];
//				}					speed_r = 0.8228f * power_r;
//				speed_l = 0.9239f * power_l;
//
//				if(speed_l >= 0)
//					sign = 1;
//				else if(speed_l < 0)
//					sign = 0;
//
//				if(last_sign != sign){
//					motor_l.SetClockwise(sign);
//					motor_r.SetClockwise(sign);
//				}
//				last_sign = sign;
//
//				if(speed_l > 900)
//					speed_l = 900;
//				else if(speed_l < -900)
//					speed_l = -900;
//
//				if(speed_r > 900)
//					speed_r = 900;
//				else if(speed_r < -900)
//					speed_r = -900;
//
//				motor_l.SetPower(int(abs(speed_l) + 66.96f));
//				motor_r.SetPower(int(abs(speed_r) + 58.24f));
//
//				ccd_average = ccd_sum / Tsl1401cl::kSensorW;
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
//				for(int i = 0; i < Tsl1401cl::kSensorW; i++){
//					if(Data[i] < ccd_average)
//						Data[i] = 0;
//					else
//						{
//						Data[i] = 1;
//						white_count++;
//						}
//				}
//
//				// left is 0
//				if(Data[63] == 1){
//					for (a = a + 1; a < Tsl1401cl::kSensorW; a++)
//					{
//						if(Data[a] == 1)
//							border_r = a;
//						else
//							break;
//					}
//
//					for(b = b - 1; b >= 0; b--)
//					{
//						if(Data[b] == 1)
//							border_l = b;
//						else
//							break;
//					}
//
////						while(border_r - border_l < 80)
////						{
////							for (a = a + 1; a < Tsl1401cl::kSensorW; a++)
////							{
////								if(Data[a] == 60)
////									border_r = a;
////								else
////									break;
////							}
////
////							for(b = b - 1; b >= 0; b--)
////							{
////								if(Data[b] == 60)
////									border_l = b;
////								else
////									break;
////							}
////						}
//				}
//
//				else
//				{
//					if(mid_point >= 63)
//					{
//						for(int i = 64; i < Tsl1401cl::kSensorW; i++)
//						{
//							if (Data[i] == 1)
//							{
//								border_l = i;
//								border_r = Tsl1401cl::kSensorW - 1;
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
//						for(int i = 62; i >= 0; i--)
//						{
//							if (Data[i] == 1)
//							{
//								border_r = i;
//								border_l = 0;
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
//				float last_turn_error = turn_error;
//				turn_error = 63 - mid_point;
//
//				float turn_error_change = turn_error - last_turn_error;
//				float hehe = turn_Kp->GetReal();
//				if(white_count >= 105)
//					hehe = hehe / 10;
//
//				if(turn_error > 20 || turn_error < -20)
//					hehe = hehe * 1.5;
//
////				if(turn_error > 0){
////					turn[0] = 1 - hehe * turn_error - turn_error_change * turn_Kd->GetReal();
////					if(turn[0] < 0)
////						turn[0] = 0;
////				}
////				else{1
////					turn[1] = 1 + hehe * 0.75 * turn_error + turn_error_change * turn_Kd->GetReal();
////					if(turn[1] < 0)
////						turn[1] = 0;
////				}
//
//				turn = hehe * turn_error + turn_error_change * turn_Kd->GetReal();
//
//
//				if(print_data){
//					St7735r::Rect rect_1;
//					for(int i = 0; i<Tsl1401cl::kSensorW; i++){
//						rect_1.x = i;
//						rect_1.y = Data[i];
//						rect_1.w = 1;
//						rect_1.h = 1;
//						lcd.SetRegion(rect_1);
//						lcd.FillColor(~0);
//					}
//				}
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

//				accel = mpu6050.GetAccel();
//				omega = mpu6050.GetOmega();
//
//				raw_accel = mpu6050.GetAccel();
//				raw_accel_angle = raw_accel[0] * 57.29578;
//				last_accel_angle = accel_angle;
//				double temp = 0;
//				acc.Filtering(&temp, (double)accel[0], 0);
//				accel[0] = temp;
//				accel_angle = accel[0]*57.29578;
//				accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;
//
//				last_gyro_angle = gyro_angle;
//				gyro_angle += -omega[1] * 0.0035 + trust_accel * (accel_angle - gyro_angle);
//				//			gyro_angle = accel_angle+(-1) *omega[0];
//				//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;
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
//				ideal_count = (int32_t)(ic_Kp * now_angle_error  + ic_Kd * angle_error_change - still_Ki * total_count_r);
			}


			if((int32_t)(t-pt3) >= 1 && yo == 3){
				//				lincoln.Turn();
				pt4 = System::Time();

				//				pt1 = t;
				yo = 4;

				encoder_r.Update();
				encoder_l.Update();

				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;

				speed = (count_l + count_r)/ 2;

				last_speed_error = speed_error;
				speed_error = ideal_speed - speed;
			    speed_error_change = (speed_error - last_speed_error);

//			    if(ideal_speed->GetReal() != 0)
//			    	anti_friction_angle = 0.5;
			    original_angle = raw_angle + anti_friction_angle - is_Kp * speed_error - is_Kd * speed_error_change;
			    anti_friction_angle = 0;

//			    if(turn[0] < 0.1 || turn[1] < 0.1)
//			    	 original_angle = raw_angle;

					last_ir_encoder_error = ir_encoder_error;
					ir_encoder_error = ideal_count - count_r;
					last_il_encoder_error = il_encoder_error;
					il_encoder_error = ideal_count - count_l;
					il_encoder_error_change = il_encoder_error - last_il_encoder_error;
					ir_encoder_error_change = ir_encoder_error - last_ir_encoder_error;

					ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
					il_encoder_errorsum += (float)il_encoder_error * 0.001;

//					speed_PID = ideal_speed + speed_Kp * speed_error + speed_Kd * speed_error_change;
//
//					if(speed_Kp == 0)
//						speed_PID = 0;
//
//					power_r = (float)ideal_count + speed_PID + (float)ir_encoder_error * encoder_r_Kp->GetReal() + ir_encoder_error_change * encoder_r_Kd->GetReal();
//					power_l = (float)ideal_count + speed_PID + (float)il_encoder_error * encoder_l_Kp->GetReal() + il_encoder_error_change * encoder_l_Kd->GetReal();
//
//					if(turn > 0)
//						power_r += turn;
//					else
//						power_l -= turn;
//
//					speed_r = 1.8978f * power_r;
//					speed_l = 1.6353f * power_l;
//
//					if(speed_l >= 0)
//						sign = 0;
//					else if(speed_l < 0)
//						sign = 1;
//
//					motor_l.SetClockwise(sign);
//					motor_r.SetClockwise(sign);
//
//					if(speed_l > 900)
//						speed_l = 900;
//					else if(speed_l < -900)
//						speed_l = -900;
//
//					if(speed_r > 900)
//						speed_r = 900;
//					else if(speed_r < -900)
//						speed_r = -900;
//
//					motor_l.SetPower(int(abs(speed_l) + 49.50f));
//					motor_r.SetPower(int(abs(speed_r) + 50.20f));
			}


			if((int32_t)(t-pt4) >= 1 && yo == 4){
				pt5 = System::Time(); //?
				yo =0;
				encoder_r.Update();
				encoder_l.Update();

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

				printf("%f, %d, %f, %d\n",lincoln1, count_l, lincoln1, count_r);

				lincoln1++;

				motor_l.SetClockwise(0);
				motor_r.SetClockwise(0);

				motor_l.SetPower(lincoln1);
				motor_r.SetPower(0);
			}

//			if((int)(t - pt7) >= 1000)
//			{
//				pt7 = t;
//
//				// Change print mode while running
//				if (button0.IsDown())
//				{
//					led0.Switch();
//					print_data = !print_data;
//				}
//
//
//				// Enable print mode without running
//				else if(button1.IsDown())
//				{
//					led1.Switch();
//					float average = 0;
//					while(1)
//					{
//						t = System::Time();
//
//						if((int)(t - pt) >= 6)
//						{
//							pt = t;
//							ccd_counter++;
//
//							ccd.StartSample();
//							while (!ccd.SampleProcess()){}
//							Data = ccd.GetData();
//
//							uint32_t sum = 0;
//
//							for(int i = 0; i < Tsl1401cl::kSensorW; i++){
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
//							for(int i = 0; i<Tsl1401cl::kSensorW; i++){
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
//							for(int i=0; i<Tsl1401cl::kSensorW; i++){
//								if(Data[i] < average)
//									Data[i] = 0;
//								else
//									Data[i] = 60;
//							}
//
//							for(int i = 0; i<Tsl1401cl::kSensorW; i++){
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
//						// Break the mode that print pixels without running
//						if((int)(t - pt8) >= 1000){
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
//		int count_l =  - encoder_l.GetCount();
//		int count_r = encoder_r.GetCount();
//
//
//	}

