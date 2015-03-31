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


float ideal_count_Kd = 0;
float ideal_count_Kp = 0;
float error_kd = 0;
//float ic_Kp = 50;
//float ic_Kd = 0.035;
float gyro_Ki = 0;

float turn_Kp = 0;
float turn_Ki = 0;
float turn_Kd = 0;

float encoder_r_Kp = 1.92;     // Recommend 1.85
float encoder_r_Ki = 0.0005;   // Recommend 0.0005
float encoder_l_Kp = 1.82;     // Recommend 1.85
float encoder_l_Ki = 0.0005;   // Recommend 0.0005
float original_angle = 0;
float new_original_angle = 0;
float turn[2] = { 1, 1 };
float still_Ki = 0.000;
float ratio_old = 0;
float ratio_new = 1-ratio_old;
int32_t first_count = 0;
float trust_accel = 0.01;
float trust_old_accel = 0;
float trust_new_accel = 1- trust_old_accel;
float power_l = 0.0f;
float power_r = 0.0f;

int32_t count_l =0;
int32_t count_r =0;

float lincoln1 = 0;

int32_t ideal_count = 0;


int main()
{
	std::array<float, 3>accel;
	std::array<float, 3>angle;
	std::array<float, 3>omega;

	//intialize the system
	System::Init();
	Timer::TimerInt t = 0;
	Timer::TimerInt pt = t;
	pt = System::Time();

	RemoteVarManager* varmanager = new RemoteVarManager(3);

	//	Initalize the BT module
	JyMcuBt106::Config bt_config;
	bt_config.id = 0;
	bt_config.rx_irq_threshold = 2;
	// Set the baud rate (data transmission rate) to 115200 (this value must
	// match the one set in the module, i.e., 115200, so you should not change
	// here, or you won't be able to receive/transmit anything correctly)
	bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	bt_config.rx_isr = std::bind(&RemoteVarManager::OnUartReceiveChar, varmanager, std::placeholders::_1);
	JyMcuBt106 bt(bt_config);


	libutil::InitDefaultFwriteHandler(&bt);

	RemoteVarManager::Var* ic_Kp = varmanager->Register("ic_Kp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* ic_Kd = varmanager->Register("ic_Kd",RemoteVarManager::Var::Type::kReal);
//	RemoteVarManager::Var* speed = varmanager->Register("speed",RemoteVarManager::Var::Type::kInt);

	printf("ic_Kp,real,0,50\n");
	printf("ic_Kd,real,1,0\n");
//	printf("speed,int,2,0\n");


	//	FtdiFt232r::Config uart_config;
	//	uart_config.id = 0;
	//	uart_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	//	FtdiFt232r fu(uart_config);

	//	Adc::Config Config;
	//	Config.adc = Adc::Name::kAdc1Ad5B;
	//	Config.resolution = Adc::Config::Resolution::k16Bit;
	//	Adc LCCD(Config);

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

	double R[2] = {0.0001, -1};

	Kalman Kalman_l(0.000001, R, 0, 1);
	Kalman Kalman_r(0.000001, R, 0, 1);

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

	System::DelayMs(25);

	motor_l.SetPower(0);
	motor_r.SetPower(0);
	count_l = 0;
	count_r = 0;


	float raw_angle;
	t= System::Time();
	pt = t;
	while(1){

		mpu6050.Update();
		System::DelayMs(4);
		accel = mpu6050.GetAccel();
		raw_angle = accel[0]*57.29578;

		t = System::Time();
		if((t-pt)>=2000)
			break;
	}

	original_angle = raw_angle;

	float accel_angle = original_angle;
	float last_gyro_angle = original_angle;
	float gyro_angle = original_angle;
	float last_accel_angle = original_angle;
	float output_angle ;            //karmen filtered

	float total_count_l =0;
	float total_count_r =0;
	float last_angle_error = 0;
	float now_angle_error = 0;
	float angle_error_change = 0;

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

	int sign = 0;
	int last_sign = 0;
	int last_ideal_count = 0;

	Byte yo = 0;                      //to organize the sequence of code

	encoder_r.Update();               //to reset the count
	encoder_l.Update();

	encoder_r.Update();               //to reset the count
	encoder_l.Update();
	while(1){

		if(t !=System::Time()){
			t = System::Time();

			if(total_count_r >= 10000)
				total_count_r = 10000;
			if(total_count_r <= -10000)
				total_count_r = -10000;
			if(total_count_l >= 10000)
				total_count_l = 10000;
			if(total_count_l <= -10000)
				total_count_l = -10000;

			if(t - pt6 >= 1)
			{
				ccd.SampleProcess();
			}

			if((int32_t)(t-pt1) >= 9  && yo==0){
				//				//				lincoln.Turn();
				pt1 = System::Time();
				//
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

				ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
				il_encoder_errorsum += (float)il_encoder_error * 0.001;


				mpu6050.Update();

				accel = mpu6050.GetAccel();
				omega = mpu6050.GetOmega();

				last_accel_angle = accel_angle;
				accel_angle = accel[0]*57.29578;
				accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;

				last_gyro_angle = gyro_angle;
				gyro_angle += (-1) * omega[1] * 0.005 + trust_accel * (accel_angle - gyro_angle);

				//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
				//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
				//				output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;

				output_angle = gyro_angle;
				last_angle_error = now_angle_error;
				now_angle_error = original_angle - output_angle;
				angle_error_change = now_angle_error -last_angle_error;

				int angle_gain = 1;
//				if(now_angle_error > 4 || now_angle_error < -4)
//					angle_gain = 3;
//				else if(now_angle_error > 7 || now_angle_error < -6)
//					angle_gain = 50;
//				else if(now_angle_error > 10 || now_angle_error < -8)
//					angle_gain = 800;
//				else if(now_angle_error > 16 || now_angle_error < -14)
//					angle_gain = 1100;

				last_ideal_count = ideal_count;
				ideal_count = (int32_t)(ic_Kp->GetReal() * now_angle_error * angle_gain + angle_gain * ic_Kd->GetReal() * angle_error_change / 0.003 - still_Ki * total_count_r);
//				ideal_count = (int32_t)(0.2*last_ideal_count + 0.8*ideal_count);

				if(now_angle_error > -0.2 && now_angle_error < 0.2)
					ideal_count = 0;

				angle_gain = 1;


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

				last_sign = sign;
				if(ideal_count > 0){
					sign = 0;
				}
				else if(ideal_count < 0){
					sign = 1;
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

					ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
					il_encoder_errorsum += (float)il_encoder_error * 0.001;


					float hehe = 1;
//					if(ideal_count > 400 || ideal_count < -400)
//						hehe = 8.0f;
//					else if (ideal_count < 40 || ideal_count > -40)
//						hehe = 0.2f;
//					else if(ideal_count > 600 || ideal_count < -600)
//						hehe = 40.0f;

					power_r = (float)ideal_count + (float)ir_encoder_error * encoder_r_Kp + ir_encoder_errorsum * encoder_r_Ki;
					power_l = (float)ideal_count + (float)il_encoder_error * encoder_l_Kp + il_encoder_errorsum * encoder_l_Ki;

					speed_l = 0.54f * power_l + 11.0f;
					speed_r = 0.65f * power_r + 28.0f;

					hehe = 1;


					if(speed_l > 900 && sign ==0){
						speed_l = 900;
					}
					else if(speed_l < -900 && sign ==1){
						speed_l = -900;
					}


					if(speed_r > 900 && sign ==0){
						speed_r = 900;
					}
					else if(speed_r < -900 && sign ==1){
						speed_r = -900;
					}

					speed_r = speed_r * turn[1];
					speed_l = speed_l * turn[0];


					motor_l.SetPower(abs(speed_l));
					motor_r.SetPower(abs(speed_r));


				}


			}

			if((int32_t)(t-pt2) >= 2 && yo == 2){
//				howard.Set(1);
				pt0 = System::Time();
				yo = 8;

				encoder_r.Update();
				encoder_l.Update();

				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				total_count_l += (float)count_l * 0.002;
				total_count_r += (float)count_r * 0.002;

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count - count_r;
				last_il_encoder_error = il_encoder_error;
				il_encoder_error = ideal_count - count_l;

				ir_encoder_errorsum += (float)ir_encoder_error * 0.002;
				il_encoder_errorsum += (float)il_encoder_error * 0.002;


				int a = 63, b = 64;
				int border_r = 0, border_l = 0, mid_point = 0;;


				if(ccd.IsImageReady())
				{
//					System::DelayMs(5);
					ccd.StartSample();
					std::array<uint16_t, Tsl1401cl::kSensorW> Data = ccd.GetData();  // 0 - 127 is left to right from the view of CCD
					uint32_t ccd_sum = 0;

					for(int i = 0; i < Tsl1401cl::kSensorW; i++){
						Data[i] = (float)Data[i] * 80.0f / 65535.0f;
						ccd_sum += Data[i];
					}

//					St7735r::Rect rect_1, rect_2, rect_3, rect_4;
//					for(int i = 0; i<Tsl1401cl::kSensorW; i++){
//						rect_1.x = i;
//						rect_1.y = 0;
//						rect_1.w = 1;
//						rect_1.h = Data[i];
//						rect_2.x = i;
//						rect_2.y = Data[i];
//						rect_2.w = 1;
//						rect_2.h = 80 - Data[i];
//						lcd.SetRegion(rect_1);
//						lcd.FillColor(~0);
//						lcd.SetRegion(rect_2);
//						lcd.FillColor(0);
//					}

					uint16_t ccd_average = ccd_sum / Tsl1401cl::kSensorW;

					if(ccd_average > 70)
						ccd_average = 722;
					else if(ccd_average < 35)
						ccd_average = 35;
					else
						ccd_average = ccd_average + 3;

					for(int i=0; i < Tsl1401cl::kSensorW; i++){
						if(Data[i] < ccd_average)
							Data[i] = 0;
						else
							Data[i] = 60;
					}

//					for(int i = 0; i<Tsl1401cl::kSensorW; i++){
//						rect_3.x = i;
//						rect_3.y = 90;
//						rect_3.w = 1;
//						rect_3.h = Data[i];
//						rect_4.x = i;
//						rect_4.y = 90 + Data[i];
//						rect_4.w = 1;
//						rect_4.h = 60 - Data[i];
//						lcd.SetRegion(rect_3);
//						lcd.FillColor(~0);
//						lcd.SetRegion(rect_4);
//						lcd.FillColor(0);
//					}

//					if(Data[64] == 60){
//						for (a = a + 1; a < Tsl1401cl::kSensorW; a++)
//						{
//							if(Data[a] == 60)
//								border_r = a;
//							else
//								break;
//						}
//
//						for(b = b - 1; b >= 0; b--)
//						{
//							if(Data[b] == 60)
//								border_l = b;
//							else
//								break;
//						}
//
//
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
//					}
//
//					else
//					{
//						if(mid_point > 64)
//						{
//							for(int i = 64; i < Tsl1401cl::kSensorW; i++)
//							{
//								if (Data[i] == 60)
//								{
//									border_l = i;
//									border_r = Tsl1401cl::kSensorW;
//									break;
//								}
//								else{
//									border_l = i;
//									border_r = i;
//								}
//							}
//						}
//
//						else
//						{
//							for(int i = 63; i > 0; i--)
//							{
//								if (Data[i] == 60)
//								{
//									border_r = i;
//									border_l = 0;
//									break;
//								}
//								else{
//									border_l = i;
//									border_r = i;
//								}
//							}
//						}
//					}

					mid_point = (border_l + border_r) / 2;
					float turn_error = Tsl1401cl::kSensorW / 2 - mid_point;

//					if(turn_error > 0){
//						turn[0] = 1 - turn_Kp * turn_error;
//						if(turn[0] < 0)
//							turn[0] = 0;
//					}
//					else{
//						turn[1] = 1 - turn_Kp * turn_error;
//						if(turn[1] < 0)
//							turn[1] = 1;
//					}

				}

			}



			/*second round to get angle and encoder
			 *
			 *
			 *
			 *
			 */
			if((int32_t)(t-pt0) >= 2  && yo == 8){
				//				lincoln.Turn();
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

				total_count_l += (float)count_l * 0.002;
				total_count_r += (float)count_r * 0.002;

				ir_encoder_errorsum += (float)ir_encoder_error * 0.002;
				il_encoder_errorsum += (float)il_encoder_error * 0.002;

				mpu6050.Update();

				accel = mpu6050.GetAccel();
				omega = mpu6050.GetOmega();

				last_accel_angle = accel_angle;
				accel_angle = accel[0]*57.29578;
				accel_angle = trust_old_accel*last_accel_angle +trust_new_accel*accel_angle;

				last_gyro_angle = gyro_angle;
				gyro_angle += (-1) *omega[1]*0.005 + trust_accel * (accel_angle - gyro_angle);
				//			gyro_angle = accel_angle+(-1) *omega[0];
				//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;



				//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
				//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
				//				output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;

				output_angle = gyro_angle;
				last_angle_error = now_angle_error;
				now_angle_error = original_angle - output_angle;
				angle_error_change = now_angle_error -last_angle_error;

				int angle_gain = 1;
//				if(now_angle_error > 4 || now_angle_error < -4)
//					angle_gain = 3;
//				else if(now_angle_error > 7 || now_angle_error < -6)
//					angle_gain = 50;
//				else if(now_angle_error > 10 || now_angle_error < -8)
//					angle_gain = 800;
//				else if(now_angle_error > 16 || now_angle_error < -14)
//					angle_gain = 1100;

				last_ideal_count = ideal_count;
				ideal_count = (int32_t)(ic_Kp->GetReal() * now_angle_error * angle_gain + angle_gain * ic_Kd->GetReal() * angle_error_change / 0.003 - still_Ki * total_count_r);
//				ideal_count = (int32_t)(0.2 * last_ideal_count + 0.8 * ideal_count);

				if(now_angle_error > -0.2 && now_angle_error < 0.2)
					ideal_count = 0;
				angle_gain = 1;

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

//				double temp = 0;
//				Kalman_l.Filtering(&temp, count_l, 0.0);
//				count_l = temp;
//				Kalman_r.Filtering(&temp, count_r, 0.0);
//				count_r = temp;

				total_count_l += (float)count_l * 0.001;
				total_count_r += (float)count_r * 0.001;

				last_sign = sign;
				if(ideal_count > 0){
					sign = 0;
				}
				else if(ideal_count < 0){
					sign = 1;
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

					ir_encoder_errorsum += (float)ir_encoder_error * 0.001;
					il_encoder_errorsum += (float)il_encoder_error * 0.001;

					float hehe = 1;
//					if(ideal_count > 400 || ideal_count < -400)
//						hehe = 8.0f;
//					else if (ideal_count < 60 || ideal_count > -60)
//						hehe = 0.2f;
//					else if(ideal_count > 600 || ideal_count < -600)
//						hehe = 40.0f;

					power_r = (float)ideal_count + (float)ir_encoder_error * encoder_r_Kp + ir_encoder_errorsum * encoder_r_Ki;
					power_l = (float)ideal_count + (float)il_encoder_error * encoder_l_Kp + il_encoder_errorsum * encoder_l_Ki;

					speed_l = 0.54f * power_l + 11.0f;
					speed_r = 0.65f * power_r + 28.0f;

					hehe = 1;

					if(speed_l > 900 && sign == 0){
						speed_l = 900;
					}
					else if(speed_l < -900 && sign ==1){
						speed_l = -900;
					}


					if(speed_r > 900 && sign ==0){
						speed_r = 900;
					}
					else if(speed_r < -900 && sign ==1){
						speed_r = -900;
					}

					speed_r = speed_r * turn[1];
					speed_l = speed_l * turn[0];


					motor_l.SetPower(abs(speed_l));
					motor_r.SetPower(abs(speed_r));


				}


			}


			if((int32_t)(t-pt4) >= 2 && yo == 4){
				pt5 = System::Time();
				yo =0;
				encoder_r.Update();
				encoder_l.Update();

				count_r = (int32_t)(-encoder_r.GetCount());
				count_l = (int32_t)(encoder_l.GetCount());

				total_count_l += (float)count_l * 0.002;
				total_count_r += (float)count_r * 0.002;

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count - count_r;
				last_il_encoder_error = il_encoder_error;
				il_encoder_error = ideal_count - count_l;

				ir_encoder_errorsum += (float)ir_encoder_error * 0.002;
				il_encoder_errorsum += (float)il_encoder_error * 0.002;

//				switch(turn_l->GetInt())
//				{
//				case 1:
//					turn[0] = 0.5;
//					break;
//				default:
//					turn[0] = 1;
//					break;
//				}
//
//				switch(turn_r->GetInt())
//				{
//				case 1:
//					turn[1] = 0.5;
//					break;
//				default:
//					turn[1] = 1;
//					break;
//				}
//
//				switch(speed->GetInt())
//				{
//				case 1:
//					original_angle = raw_angle + 1;
//					break;
//				case 2:
//					original_angle = raw_angle - 1;
//					break;
//				default:
//					original_angle = raw_angle;
//					break;
//				}


				printf("%f, %d, %f, %f\n", output_angle, ideal_count, ic_Kp->GetReal(), ic_Kd->GetReal());

			}



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

