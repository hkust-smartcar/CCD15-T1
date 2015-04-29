/*
 * lincoln_main.h
 *
 *  Created on: 2 Mar, 2015
 *      Author: lincoln
 */

#include <cmath>
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
#define rad_to_degree    57.29578

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
float ic_Kd = 0.6;      //9.1
float ic_Kp = 0;
float ic_Kp_const = 2.9;    //2.51
float moving_gain = 39.6;      //2.7
float ic_Ki = 0;
float mg = 0;

float gyro_Ki = 0;
//float encoder_Kp = 204;
//float encoder_Kd = 81;
float encoder_r_Kp = 5.32;      //18.53   8.05v  //21.73       31         7.79V
float encoder_r_Kd = 0;       //2.01
float encoder_r_Ki = 0;
float encoder_l_Kp = 2.8;    //22.05   8.05v    //19.05                          28.9          7.76V
float encoder_l_Kd = 0;      //0.68
float encoder_l_Ki = 0;

float turning_Kp = 0;
float turning_Kd = 0;
float turning_Ki = 0;
int32_t turning_count = 0;

float isKp = 0;
float isKd = 0;

float original_angle = 0;           //-12.24
float new_original_angle = 0;
float turn[2] = { 1, 1 };
float still_Kp = 0;
float still_Kd = 0;
float ratio_old = 0;
float ratio_new = 1-ratio_old;
int32_t first_count = 0;
float trust_accel = 0.012;             //0.021
float trust_old_accel = 0.83;
float trust_new_accel = 1- trust_old_accel;
int l_edge = 0;
int r_edge = 0;
Byte l_color_flag = 0;
Byte r_color_flag = 0;
Byte blue_flag = 0;
Byte get_sample_flag = 0;

//float howard =0;
float lincoln1 = 0;

int32_t ideal_count = 0;
float gyro_in_time = 0.0041;
void myListener(const std::vector<Byte> &bytes)
{
	switch (bytes[0])
	{
	case'1':

		original_angle -= 0.01;

		break;
	case '2':
		original_angle += 0.01;
		break;
	case '!':

		original_angle -= 0.5;

		break;
	case '@':
		original_angle += 0.5;
		break;
	case'5':

		trust_accel -= 0.001;

		break;
	case '6':
		trust_accel += 0.001;
		break;
	case '7':

		turning_count = 2;

		break;

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
		//	case '1':
		//		if(ideal_count >= 5){
		//			ideal_count -= 5;
		//		}
		//		break;
		//	case '2':
		//		ideal_count += 5;
		//		break;
		//	case '!':
		//		if(ideal_count >= 20){
		//			ideal_count -= 20;
		//		}
		//		break;
		//	case '@':
		//		ideal_count += 20;
		//		break;
		//	case '8':
		//		trust_accel += 0.001;
		//		break;
		//	case '*':
		//		trust_accel -=0.001;
		//		break;
		//	case '6':
		//		gyro_in_time += 0.001;
		//		break;
		//	case '^':
		//		gyro_in_time -=0.001;
		//		break;
		//	case '0':
		//		turn[0] = 1.3;
		//		turn[1] = 0;
		//		break;
		//			case '9':
		//				if(trust_accel > 0.001){
		//					trust_accel -= 0.001;
		//				}
		//				break;
		//			case '0':
		//				trust_accel += 0.001;
		//				break;
		//			case '(':
		//				if(trust_accel > 0.01){
		//					trust_accel -= 0.01;
		//				}
		//				break;
		//			case ')':
		//				trust_accel += 0.01;
		//				break;


	case '9':
		if(turning_Kp > 0.001){
			turning_Kp -= 0.001;
		}
		break;
	case '0':
		turning_Kp += 0.001;
		break;
	case '(':
		if(turning_Kp > 0.01){
			turning_Kp -= 0.01;
		}
		break;
	case ')':
		turning_Kp += 0.01;
		break;
	case 'h':
		if(turning_Kd > 0.1){
			turning_Kd -= 0.1;
		}
		break;
	case 'j':
		turning_Kd += 0.1;
		break;
	case 'H':
		if(turning_Kd > 1){
			turning_Kd -= 1;
		}
		break;
	case 'J':
		turning_Kd += 1;
		break;
	case 'k':
		if(turning_Ki > 0.01){
			turning_Ki -= 0.01;
		}
		break;
	case 'l':
		turning_Ki += 0.01;
		break;
	case 'K':
		if(turning_Ki > 0.1){
			turning_Ki -= 0.1;
		}
		break;
	case 'L':
		turning_Ki += 0.1;
		break;
		//	case 'z':
		//		if(turning_Kp > 0.1){
		//			turning_Kp -= 0.1;
		//		}
		//		break;
		//	case 'x':
		//		turning_Kp += 0.1;
		//		break;
		//	case 'Z':
		//		if(turning_Kp > 1){
		//			turning_Kp -= 1;
		//		}
		//		break;
		//	case 'X':
		//		turning_Kp += 1;
		//		break;
		//	case 'c':
		//		if(turning_Kd > 0.05){
		//			turning_Kd -= 0.05;
		//		}
		//		break;
		//	case 'v':
		//		turning_Kd += 0.05;
		//		break;
		//	case 'C':
		//		if(turning_Kd > 0.1){
		//			turning_Kd -= 0.1;
		//		}
		//		break;
		//	case 'V':
		//		turning_Kd += 0.1;
		//		break;
	case 'a':
		if(ic_Kp_const > 0.05){
			ic_Kp_const -= 0.05;
		}
		break;
	case 's':
		ic_Kp_const += 0.05;
		break;
	case 'A':
		if(ic_Kp_const > 0.5){
			ic_Kp_const -= 0.5;
		}
		break;
	case 'S':
		ic_Kp_const += 0.5;
		break;
	case 'd':
		if(ic_Kd > 0.01){
			ic_Kd -= 0.01;
		}
		break;
	case 'f':
		ic_Kd += 0.01;
		break;
	case 'D':
		if(ic_Kd > 1){
			ic_Kd -= 1;
		}
		break;
	case 'F':
		ic_Kd += 1;
		break;
	case 'u':
		if(encoder_l_Kp > 0.1){
			encoder_l_Kp -= 0.1;
		}
		break;
	case 'i':
		encoder_l_Kp += 0.1;
		break;
	case 'U':
		if(encoder_l_Kp > 1){
			encoder_l_Kp -= 1;
		}
		break;
	case 'I':
		encoder_l_Kp += 1;
		break;
	case 'o':
		if(encoder_r_Kp > 0.01){
			encoder_r_Kp -= 0.01;
		}
		break;
	case 'p':
		encoder_r_Kp += 0.01;
		break;
	case 'O':
		if(encoder_r_Kp > 0.1){
			encoder_r_Kp -= 0.1;
		}
		break;
	case 'P':
		encoder_r_Kp += 0.1;
		break;
		//	case 'f':
		//		if(encoder_l_Ki > 0.05){
		//			encoder_l_Ki -= 0.05;
		//		}
		//		break;
		//	case 'g':
		//		encoder_l_Ki += 0.05;
		//		break;
		//	case 'F':
		//		if(encoder_l_Ki > 1){
		//			encoder_l_Ki -= 1;
		//		}
		//		break;
		//	case 'G':
		//		encoder_l_Ki += 1;
		//		break;


	}
}


int main()
{
	//	JyMcuBt106::Config uartConfig;
	//	uartConfig.id = 0;
	//	uartConfig.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	//	JyMcuBt106 uasdasd(uartConfig);
	//
	//		Gpo::Config gpoConfig;
	//		gpoConfig.pin = LIBSC_UART0_RX;
	//		gpoConfig.is_high = true;
	//	//
	//		Gpo a(gpoConfig);
	//
	//	gpoConfig.pin = libbase::k60::Pin::Name::kPtb1;
	//	Gpo b(gpoConfig);
	//
	//		while (true);

	System::Init();
	//			JyMcuBt106::Config uart_config;
	//			uart_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	//			//uart_config.rx_irq_threshold = 7;
	//			//uart_config.is_rx_irq_threshold_percentage = false;
	//			//uart_config.tx_buf_size = 50;
	//			JyMcuBt106 fu(uart_config);

	//		while (true)
	//		{
	//			fu.SendStrLiteral("hello\n");
	//			System::DelayMs(250);
	//			char b;
	//			if (fu.PeekChar(&b))
	//			{
	//				fu.SendBuffer((Byte*)&b, 1);
	//			}
	//		}

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

//			JyMcuBt106::Config uart_config;
//			uart_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//			uart_config.rx_irq_threshold = 7;
//			uart_config.is_rx_irq_threshold_percentage = false;
//			uart_config.tx_buf_size = 50;
//			JyMcuBt106 fu(uart_config);

	//		while(1);
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
	int16_t center_line = 0;
	int16_t center_line_flag = 0;
	int16_t center_line_error = 0;
	float last_center_line_error = 0;
	float now_center_line_error = 0;
	float center_line_error_change = 0;
	int16_t road_length = 0;
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




	//	Gpo::Config howard;
	//	howard.pin = Pin::Name::kPtd1;
	//	Gpo lincoln(howard);


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
		raw_angle = accel[0]*rad_to_degree;

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
	//	original_angle = -19.1;

	float accel_angle = original_angle;
	float last_gyro_angle = original_angle;
	float gyro_angle = 0;//original_angle;
	float last_accel_angle = original_angle;
	float output_angle = 0;            //karmen filtered
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
	Byte lcd_flag = 0;

	int32_t moving_ideal_count_1 = 0;
	int32_t moving_ideal_count_2 = 0;
	int32_t moving_ideal_count_3 = 0;
	int32_t moving_ideal_count_4 = 0;
	int32_t moving_ideal_count_5 = 0;
	int32_t moving_ideal_count_6 = 0;

	float last_travel_speed_error = 0;
	float now_travel_speed_error = 0;
	float ideal_travel_speed = 0;
	float travel_speed_error_change = 0;

	Byte moving_count_flag = 0;

	float moving_accel_1 = 0;
	float moving_accel_2 = 0;
	float moving_accel_3 = 0;
	float moving_accel_4 = 0;
	float moving_accel_5 = 0;
	float moving_accel_6 = 0;
	float moving_accel_7 = 0;
	float moving_accel_8 = 0;

	Byte moving_accel_flag = 0;

	float omegasum = 0;

	//********************************************************************************************************************
	MyVarManager pGrapher;
	//graph testing variable


	//	pGrapher.addWatchedVar(&ideal_count, "1");
	//	pGrapher.addWatchedVar(&turning_Kp, "1");
	//
	//	pGrapher.addWatchedVar(&ideal_count, "2");
	//	pGrapher.addWatchedVar(&turning_Kd, "3");
	//	pGrapher.addWatchedVar(&turning_Ki, "4");
	//		pGrapher.addWatchedVar(&ic_Kd, "5");
	//	pGrapher.addWatchedVar(&ic_Ki, "5");



	//	pGrapher.addWatchedVar(&ic_Kd, "4");
	//	pGrapher.addWatchedVar(&encoder_l_Kp, "4");
	//	pGrapher.addWatchedVar(&encoder_l_Kd, "5");

	//


	//			pGrapher.addWatchedVar(&omegasum,"5");

	pGrapher.addWatchedVar(&ideal_count,"1");
	pGrapher.addWatchedVar(&count_l,"2");
	pGrapher.addWatchedVar(&count_r,"3");

	//	pGrapher.addWatchedVar(&moving_gain,"4");
	//	pGrapher.addWatchedVar(&ic_Kp_const,"5");
	//	pGrapher.addWatchedVar(&ic_Kd,"6");


	pGrapher.addWatchedVar(&encoder_l_Kp,"4");
	pGrapher.addWatchedVar(&encoder_r_Kp,"5");
//	pGrapher.addWatchedVar(&gyro_angle,"6");
//	pGrapher.addWatchedVar(&accel_angle,"7");
//	pGrapher.addWatchedVar(&original_angle,"8");








	pGrapher.Init(&myListener);
	//					lincoln.Turn();

	//	output_angle = 0;

	std::array<uint16_t,Tsl1401cl::kSensorW> old_pixel;
	while(1){






		//		for(int i=0;i<5000;i++){
		//
		//			mpu6050.Update();
		//			omega = mpu6050.GetOmega();
		//			omegasum += omega[1];
		//			System::DelayMs(4);
		//		}
		//		omegasum = omegasum/5000.0f;
		//		int n = sprintf(buffer, "%.3f\n",omegasum);
		//		fu.SendBuffer((Byte*)buffer,n);
		//		memset(buffer, 0, n);
		//		while(1);


		//		for(int power = 0;power <1000;power++){
		//			encoder_r.Update();
		//			motor_r.SetPower(power);
		//
		//			System::DelayMs(5);
		//			encoder_l.Update();
		//			int n = sprintf(buffer, "%d , %d\n",power, -1*(int)encoder_r.GetCount());
		//			fu.SendBuffer((Byte*)buffer,n);
		//			memset(buffer, 0, n);
		//
		//
		//		}
		//		motor_l.SetPower(0);
		//		break;



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

			//			if((int32_t)(t-pt1) >= 4  && yo==0){
			//				yo = 1;
			//
			//
			//				//							lincoln.Set(1);                        //in debug mode, about 540us
			//
			//
			//				//							lincoln.Set(0);
			//
			//			}
			//

			//
			//			if((int32_t)(t-pt1) >= 1 && yo ==0){



			if(t % 4 ==0){
				//				lincoln.Set(1);                     //in debug mode, about 40 us.
				//				pt2=System::Time();

				//				pt1 = t;
				//				yo = 1;

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
				else if(ideal_count == 0){
					sign = last_sign;
				}


				//				if(sign == 2){
				//					motor_l.SetPower(0);
				//					motor_r.SetPower(0);
				//				}

				if(last_sign != sign){

					//					motor_l.SetPower(0);
					//					motor_r.SetPower(0);
					motor_l.SetClockwise(sign);
					motor_r.SetClockwise(sign);
				}

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count + turning_count - count_r;        //turning_count is positive when car needs to turn right
				last_il_encoder_error = il_encoder_error;                        //left wheel faster right wheel slower
				il_encoder_error = ideal_count - turning_count - count_l;        //turning_count is positive when car needs to turn right


				il_encoder_error_change = il_encoder_error -last_il_encoder_error;
				ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;

				old_speed_r = speed_r;
				old_speed_l = speed_l;

				ir_encoder_errorsum -= ir_encoder_error;
				il_encoder_errorsum += il_encoder_error;


				lincoln1 = (float)(ir_encoder_error_change * encoder_r_Kd/3);


				//					if((il_encoder_error_change+ir_encoder_error_change)/2 >= 15){
				//						speed_l = 2.4988*ideal_count + 31.4593;                           //data from graph,reference power
				//						speed_r = 2.38436*ideal_count + 41.1278;
				//					}
				//				speed_r = speed_r*0;
				//				speed_l = speed_l*0;
				speed_r = (int32_t)(ideal_count + (int32_t)(ir_encoder_error *encoder_r_Kp) + (int32_t)(ir_encoder_error_change * encoder_r_Kd) + ir_encoder_errorsum*encoder_r_Ki);
				speed_l = (int32_t)(ideal_count + (int32_t)(il_encoder_error *encoder_l_Kp) + (int32_t)(il_encoder_error_change * encoder_l_Kd) + il_encoder_errorsum*encoder_l_Ki);

				speed_l = 1.45751*speed_l;
				speed_r = 1.38485*speed_r;

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




				//				speed_r = ratio_old*old_speed_r + ratio_new*speed_r;
				//				speed_l = ratio_old*old_speed_l + ratio_new*speed_l;
				//
				//				speed_r = speed_r*turn[1];
				//				speed_l = speed_l*turn[0];



				motor_l.SetPower(abs(speed_l) + 60.62192);                 //2.4988,42
				motor_r.SetPower(abs(speed_r) + 95.15344);                //2.38436,72.15344

				//				lincoln.Set(0);
			}
			//			//
			//			//
			//			//
			//			//
			//			//
			//			//			/*second round to get angle and encoder
			//			//			 *
			//			//			 *
			//			//			 *
			//			//			 *
			//			//			 */
			//			//
			//			//			if((int32_t)(t-pt2) >= 1  && yo == 1){
			//			//if(t % 20 ==1 && yo == 1){
			//			//				lincoln.Turn();                          //in debug mode, about 540us
			//			//				pt3 = System::Time();
			if(t%4==1){
				//yo = 2;

				//				encoder_r.Update();
				//				encoder_l.Update();

				mpu6050.Update();                                   //in debug mode, about 452us

				accel = mpu6050.GetAccel();
				omega = mpu6050.GetOmega();

				last_accel_angle = accel_angle;
				accel_angle = -1*accel[0]*57.29578;
				//				accel_angle = asin(accel[0])*57.29578;

				if(moving_accel_flag == 0){
					moving_accel_1 = accel_angle;
					moving_accel_flag = 1;
				}
				else if(moving_accel_flag == 1){
					moving_accel_2 = accel_angle;
					moving_accel_flag = 2;
				}
				else if(moving_accel_flag == 2){
					moving_accel_3 = accel_angle;
					moving_accel_flag = 3;
				}
				else if(moving_accel_flag ==3){
					moving_accel_4 = accel_angle;
					moving_accel_flag = 4;
				}
				else if(moving_accel_flag == 4){
					moving_accel_5 = accel_angle;
					moving_accel_flag = 5;
				}
				else if(moving_accel_flag == 5){
					moving_accel_6 = accel_angle;
					moving_accel_flag = 6;
				}
				else if(moving_accel_flag == 6){
					moving_accel_7 = accel_angle;
					moving_accel_flag = 7;
				}
				else if(moving_accel_flag ==7){
					moving_accel_8 = accel_angle;
					moving_accel_flag = 0;
				}
				if(moving_accel_1 && moving_accel_2 && moving_accel_3 && moving_accel_4 && moving_accel_5 && moving_accel_6 && moving_accel_7 && moving_accel_8){  //  && moving_accel_7 && moving_accel_8
					accel_angle = (moving_accel_1 + moving_accel_2 + moving_accel_3 + moving_accel_4 + moving_accel_5 + moving_accel_6 + moving_accel_7 + moving_accel_8)/8.0f;//  + moving_accel_7 + moving_accel_8
				}

				//								accel_angle = trust_old_accel*last_accel_angle + trust_new_accel*accel_angle;

				last_gyro_angle = gyro_angle;
				gyro_angle += (float)((omega[1]-1.6685f)*gyro_in_time) + trust_accel*(accel_angle - gyro_angle);
				output_angle = gyro_angle;
				//				omega[1] = 0;
				//				omegasum += (float)(-1.0f*(omega[1]-1.6685)*gyro_in_time);

				//			gyro_angle = accel_angle+(-1) *omega[0];
				//				gyro_angle = 0.2*last_gyro_angle + 0.8*gyro_angle;




				//							kalman_filter_init(&m_gyro_kf[0], 0.01f, kalman_value, accel_angle, 1);
				//							kalman_filtering(&m_gyro_kf[0], &output_angle, &accel_angle, &gyro_angle, 1);
				//				output_angle = trust_gyro*gyro_angle +trust_accel*accel_angle;

				//				output_angle = gyro_angle;

				//				if(output_angle - raw_angle <5 && output_angle - raw_angle > -5){
				//					original_angle = 0.8*original_angle +0.2*raw_angle;
				//				}
				//				else if(output_angle - original_angle >=5){
				//					original_angle -= (output_angle - raw_angle-5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
				//				}
				//				else if(output_angle - original_angle <=-5){
				//					original_angle -= (output_angle - raw_angle + 5)*still_Kp + (output_angle - raw_angle -last_angle_error)*still_Kd;
				//				}


				last_travel_speed_error = now_travel_speed_error;
				now_travel_speed_error = ideal_travel_speed - (float)((count_l + count_r)/2.0f);
				travel_speed_error_change = now_travel_speed_error - last_travel_speed_error;




//				original_angle = now_travel_speed_error + isKp*now_travel_speed_error + isKd*travel_speed_error_change;

				last_angle_error = now_angle_error;
				now_angle_error =   output_angle - original_angle;
				angle_error_change = now_angle_error -last_angle_error;


				ic_Kp = moving_gain*now_angle_error/20.0f + ic_Kp_const;
				ideal_count = (int32_t)(original_angle + ic_Kp+ ic_Kd*angle_error_change);
				if(moving_count_flag == 0){
					moving_ideal_count_1 = ideal_count;
					moving_count_flag = 1;
				}
				else if(moving_count_flag == 1){
					moving_ideal_count_2 = ideal_count;
					moving_count_flag = 2;
				}
				else if(moving_count_flag == 2){
					moving_ideal_count_3 = ideal_count;
					moving_count_flag = 0;
				}
				//				else if(moving_count_flag ==3){
				//					moving_ideal_count_4 = ideal_count;
				//					moving_count_flag = 0;
				//				}

				if(moving_ideal_count_1 && moving_ideal_count_2 && moving_ideal_count_3){
					ideal_count = (int32_t)((moving_ideal_count_1 + moving_ideal_count_2 + moving_ideal_count_3)/3.0f);
				}
				//
				//
				//
				//
			}
			//			//
			//			//
			//			//
			//			//
			//			//
			//			//			if((int32_t)(t-pt3) >= 1 && yo == 3){
			if(t % 4 == 2){
				//				lincoln.Turn();                      //in debug mode, about 40 us.
				//				pt4 = System::Time();

				//				pt1 = t;
				//				yo = 3;
				if(get_sample_flag == 0){                  //to get pixel in 12ms period
					get_sample_flag = 1;
				}
				else if(get_sample_flag ==1){
					get_sample_flag =2;
				}
				else if(get_sample_flag ==2){
					ccd.StartSample();
					while (!ccd.SampleProcess())
					{}
					pixel = ccd.GetData();
					get_sample_flag = 0;
				}



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
				else if(ideal_count == 0){
					sign = last_sign;
				}


				//				if(sign == 2){
				//					motor_l.SetPower(0);
				//					motor_r.SetPower(0);
				//				}

				if(last_sign != sign){

					//					motor_l.SetPower(0);
					//					motor_r.SetPower(0);
					motor_l.SetClockwise(sign);
					motor_r.SetClockwise(sign);
				}

				last_ir_encoder_error = ir_encoder_error;
				ir_encoder_error = ideal_count + turning_count - count_r;        //turning_count is positive when car needs to turn right
				last_il_encoder_error = il_encoder_error;                        //left wheel faster right wheel slower
				il_encoder_error = ideal_count - turning_count - count_l;        //turning_count is positive when car needs to turn right


				il_encoder_error_change = il_encoder_error -last_il_encoder_error;
				ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;

				old_speed_r = speed_r;
				old_speed_l = speed_l;

				ir_encoder_errorsum -= ir_encoder_error;
				il_encoder_errorsum += il_encoder_error;


				lincoln1 = (float)(ir_encoder_error_change * encoder_r_Kd/3);


				//					if((il_encoder_error_change+ir_encoder_error_change)/2 >= 15){
				//						speed_l = 2.4988*ideal_count + 31.4593;                           //data from graph,reference power
				//						speed_r = 2.38436*ideal_count + 41.1278;
				//					}
				//				speed_r = speed_r*0;
				//				speed_l = speed_l*0;
				speed_r = (int32_t)(ideal_count + (int32_t)(ir_encoder_error *encoder_r_Kp) + (int32_t)(ir_encoder_error_change * encoder_r_Kd) + ir_encoder_errorsum*encoder_r_Ki);
				speed_l = (int32_t)(ideal_count + (int32_t)(il_encoder_error *encoder_l_Kp) + (int32_t)(il_encoder_error_change * encoder_l_Kd) + il_encoder_errorsum*encoder_l_Ki);

				speed_l = 1.45751*speed_l;
				speed_r = 1.38485*speed_r;

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




				//				speed_r = ratio_old*old_speed_r + ratio_new*speed_r;
				//				speed_l = ratio_old*old_speed_l + ratio_new*speed_l;
				//
				//				speed_r = speed_r*turn[1];
				//				speed_l = speed_l*turn[0];



				motor_l.SetPower(abs(speed_l) + 60.62192);                 //2.4988,42              33
				motor_r.SetPower(abs(speed_r) + 95.15344);                //2.38436,72.15344         60

				//				lincoln.Set(0);


			}
			//			//
			//			//
			//			//
			//			//
			//			//
			//			//
			//			//
			//			//
			//			//
			//			//
			if(t% 4 == 3){
				//				pt5 = System::Time();
				yo = 0;
				if(blue_flag == 0){
					//										pGrapher.sendWatchData();
					//										int n = sprintf(buffer, "%g , %g\n",omega[1], accel_angle);
					//										fu.SendBuffer((Byte*)buffer,n);
					//										memset(buffer, 0, n);
					pGrapher.sendWatchData();

					blue_flag = 1;
				}
				else if(blue_flag ==1){
					blue_flag =2;
				}
				else if(blue_flag ==2){
					blue_flag = 0;
				}


				//detect edge method***************
				if(get_sample_flag ==0){
					for(int i=0;i<128;i++){
						pixel[i]=(uint16_t)(pixel[i]*11.0f);      //1.05
						if(pixel[i] > 255){
							pixel[i] = 255;
						}
					}
					//				lincoln.Set(0);
					//				lincoln.Set(1);

					//				if(l_edge && r_edge){
					//					now_5pixel_value = pixel[r_edge - 10] + pixel[r_edge - 9] + pixel[r_edge - 8] + pixel[r_edge - 7] + pixel[r_edge - 6];
					//					for(int i=r_edge - 5; i < 118; i = i+5){
					//						last_5pixel_value = now_5pixel_value;
					//						now_5pixel_value = pixel[i] + pixel[i+1] + pixel[i+2] + pixel[i+3] + pixel[i+4];
					//						pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5;
					//
					//						if(i == r_edge - 5){
					//							pixel_avg_difference = pixel_difference_sum/((i-r_edge+5)/5);
					//							if(pixel_difference_sum >= 1500){
					//								pixel_avg_difference = 1;
					//							}
					//
					//						}
					//
					//						if(pixel_avg_difference >= 10){
					//							pixel_avg_difference = 1;
					//						}
					//						if((now_5pixel_value - last_5pixel_value)/5 < -150*aabbss(pixel_avg_difference)){
					//							r_edge = i;
					//							r_color_flag = white_black;
					//							pixel_difference_sum = 0;
					//							break;
					//						}
					//
					//
					//						else if((now_5pixel_value - last_5pixel_value)/5 > 150*aabbss(pixel_avg_difference)){
					//							r_edge = i;
					//							r_color_flag = black_white;
					//							pixel_difference_sum = 0;
					//							break;
					//						}
					//						else if(i == 117){
					//							if((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 > 60000){
					//								r_color_flag = half_white;
					//								pixel_difference_sum = 0;
					//								break;
					//							}
					//							else if((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 < 35000){
					//								r_color_flag = half_black;
					//								pixel_difference_sum = 0;
					//								break;
					//							}
					//						}
					//						pixel_avg_difference = pixel_difference_sum/((i-r_edge+5)/5);
					//					}
					//
					//					now_5pixel_value = pixel[l_edge +10] + pixel[l_edge +9] + pixel[l_edge +8] + pixel[l_edge +7] + pixel[l_edge +6];
					//					for(int i = l_edge +5; i > 8; i = i-5){
					//						last_5pixel_value = now_5pixel_value;
					//						now_5pixel_value = pixel[i] + pixel[i-1] + pixel[i-2] + pixel[i-3] + pixel[i-4];
					//						pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5;
					//
					//						if(i == l_edge +5){
					//							pixel_avg_difference = pixel_difference_sum/((l_edge + 10 -i)/5);
					//							if(pixel_difference_sum >= 1500){
					//								pixel_avg_difference = 1;
					//							}
					//						}
					//						if(pixel_avg_difference >= 10){
					//							pixel_avg_difference = 1;
					//						}
					//
					//						if((now_5pixel_value - last_5pixel_value)/5 < -150*aabbss(pixel_avg_difference)){
					//							l_edge = i;
					//							l_color_flag = white_black;
					//							pixel_difference_sum = 0;
					//							break;
					//						}
					//
					//
					//						else if((now_5pixel_value - last_5pixel_value)/5 > 150*aabbss(pixel_avg_difference)){
					//							l_edge = i;
					//							l_color_flag = black_white;
					//							pixel_difference_sum = 0;
					//							break;
					//						}
					//						else if(i == 9){
					//
					//							if((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 > 60000){
					//								l_color_flag = half_white;
					//								pixel_difference_sum = 0;
					//								break;
					//							}
					//							else if((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 < 35000){
					//								l_color_flag = half_black;
					//								pixel_difference_sum = 0;
					//								break;
					//							}
					//							break;
					//						}
					//						pixel_avg_difference = pixel_difference_sum/((l_edge + 10 -i)/5);
					//					}
					//
					//					if(center_line_flag == 0){
					//						road_length = (r_edge - l_edge);
					//						center_line = (road_length)/2+l_edge;
					//						center_line_flag++;
					//					}
					//
					//					last_center_line_error = now_center_line_error;
					//					now_center_line_error = (float)(center_line - ((r_edge - l_edge)/2+l_edge));         //if the error is positive, car need to turn right
					//					center_line_error_change = now_center_line_error - last_center_line_error;
					//					center_line_errorsum += now_center_line_error;
					//					if(center_line_errorsum > 1000){
					//						center_line_errorsum = 1000;
					//					}
					//					else if(center_line_errorsum < -1000){
					//						center_line_errorsum = -1000;
					//					}
					//
					//
					//					turning_count += (int32_t)(now_center_line_error*turning_Kp + center_line_error_change*turning_Kd + center_line_errorsum*turning_Ki);
					//				}

					//				else{
					now_5pixel_value = pixel[57] + pixel[58] + pixel[59] + pixel[60] + pixel[61];
					for(int i=62; i < 118; i = i+5){
						last_5pixel_value = now_5pixel_value;
						now_5pixel_value = pixel[i] + pixel[i+1] + pixel[i+2] + pixel[i+3] + pixel[i+4];
						pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5.0f;

						if(i == 62){
							pixel_avg_difference = pixel_difference_sum/((i-57)/5.0f);
							if(pixel_difference_sum >= 6){
								pixel_avg_difference = 1;
							}

						}

						if(pixel_avg_difference <= 1){
							pixel_avg_difference = 1;
						}
						if((now_5pixel_value - last_5pixel_value)/5.0f < (float)(-20*aabbss(pixel_avg_difference))){
							r_edge = i;
							r_color_flag = white_black;
							pixel_difference_sum = 0;
							break;
						}


						else if((now_5pixel_value - last_5pixel_value)/5.0f > (float)(20*aabbss(pixel_avg_difference))){
							r_edge = i;
							r_color_flag = black_white;
							pixel_difference_sum = 0;
							break;
						}
						else if(i == 117){
							if((int)((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6) > 230){
								r_edge = l_edge + road_length;
								r_color_flag = half_white;
								pixel_difference_sum = 0;
								break;
							}
							else if((int)((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 )< 100){
								r_edge = 62;
								r_color_flag = half_black;
								pixel_difference_sum = 0;
								break;
							}
						}
						pixel_avg_difference = pixel_difference_sum/((i-57)/5.0f);
					}

					now_5pixel_value = pixel[65] + pixel[66] + pixel[67] + pixel[68] + pixel[69];
					for(int i = 69; i > 8; i = i-5){
						last_5pixel_value = now_5pixel_value;
						now_5pixel_value = pixel[i] + pixel[i-1] + pixel[i-2] + pixel[i-3] + pixel[i-4];
						pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5.0f;

						if(i == 69){
							pixel_avg_difference = pixel_difference_sum/((74-i)/5.0f);
							if(pixel_difference_sum >= 6){
								pixel_avg_difference = 1;
							}
						}
						if(pixel_avg_difference <= 1){
							pixel_avg_difference = 1;
						}

						if((now_5pixel_value - last_5pixel_value)/5.0f < (float)(-20*aabbss(pixel_avg_difference))){
							l_edge = i;
							l_color_flag = white_black;
							pixel_difference_sum = 0;
							break;
						}


						else if((now_5pixel_value - last_5pixel_value)/5.0f > (float)(20*aabbss(pixel_avg_difference))){
							l_edge = i;
							l_color_flag = black_white;
							pixel_difference_sum = 0;
							break;
						}
						else if(i == 9){

							if((int)((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6) > 230){
								l_edge = r_edge - road_length;
								l_color_flag = half_white;
								pixel_difference_sum = 0;
								break;
							}
							else if((int)((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 )< 100){
								l_edge = 69;
								l_color_flag = half_black;
								pixel_difference_sum = 0;
								break;
							}
							break;
						}
						pixel_avg_difference = pixel_difference_sum/((74 - i)/5.0f);
					}

					if(l_color_flag == half_white && r_color_flag == half_white){
						now_center_line_error = 0;

					}

					if(center_line_flag == 0 && l_edge > 0 && r_edge >0){
						road_length = (int16_t)(r_edge - l_edge);
						center_line = (int16_t)((road_length)/2.0f)+l_edge;
						center_line_flag++;
					}

					last_center_line_error = now_center_line_error;
					now_center_line_error = (float)(center_line - ((r_edge - l_edge)/2+l_edge));         //if the error is positive, car need to turn right
					center_line_error_change = now_center_line_error - last_center_line_error;
					center_line_errorsum += now_center_line_error;
					if(center_line_errorsum > 1000){
						center_line_errorsum = 1000;
					}
					else if(center_line_errorsum < -1000){
						center_line_errorsum = -1000;
					}


					turning_count = (int32_t)(now_center_line_error*turning_Kp + center_line_error_change*turning_Kd + center_line_errorsum*turning_Ki);

				}
			}

			//
			//			//detect edge method***************
			//			//			if((int32_t)(t-pt6) >= 10 && yo == 0){
			//			if(t % 10 ==0){
			//				//				lincoln.Set(1);
			//
			//				//				yo = 0;
			//				//				lincoln.Set(1);
			//				//				System::DelayUs(100);
			//				//				for (int i = 0; i < 9; ++i)
			//				//				{
			//				//					asm("nop");
			//				//				}
			//				//				lincoln.Set(0);
			//
			//
			//
			//
			//
			//
			//				for(int i=0;i<128;i++){
			//					pixel[i]=(uint16_t)(pixel[i]*11.0f);      //1.05
			//					if(pixel[i] > 255){
			//						pixel[i] = 255;
			//					}
			//				}
			//				//				lincoln.Set(0);
			//				//				lincoln.Set(1);
			//
			//				//				if(l_edge && r_edge){
			//				//					now_5pixel_value = pixel[r_edge - 10] + pixel[r_edge - 9] + pixel[r_edge - 8] + pixel[r_edge - 7] + pixel[r_edge - 6];
			//				//					for(int i=r_edge - 5; i < 118; i = i+5){
			//				//						last_5pixel_value = now_5pixel_value;
			//				//						now_5pixel_value = pixel[i] + pixel[i+1] + pixel[i+2] + pixel[i+3] + pixel[i+4];
			//				//						pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5;
			//				//
			//				//						if(i == r_edge - 5){
			//				//							pixel_avg_difference = pixel_difference_sum/((i-r_edge+5)/5);
			//				//							if(pixel_difference_sum >= 1500){
			//				//								pixel_avg_difference = 1;
			//				//							}
			//				//
			//				//						}
			//				//
			//				//						if(pixel_avg_difference >= 10){
			//				//							pixel_avg_difference = 1;
			//				//						}
			//				//						if((now_5pixel_value - last_5pixel_value)/5 < -150*aabbss(pixel_avg_difference)){
			//				//							r_edge = i;
			//				//							r_color_flag = white_black;
			//				//							pixel_difference_sum = 0;
			//				//							break;
			//				//						}
			//				//
			//				//
			//				//						else if((now_5pixel_value - last_5pixel_value)/5 > 150*aabbss(pixel_avg_difference)){
			//				//							r_edge = i;
			//				//							r_color_flag = black_white;
			//				//							pixel_difference_sum = 0;
			//				//							break;
			//				//						}
			//				//						else if(i == 117){
			//				//							if((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 > 60000){
			//				//								r_color_flag = half_white;
			//				//								pixel_difference_sum = 0;
			//				//								break;
			//				//							}
			//				//							else if((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 < 35000){
			//				//								r_color_flag = half_black;
			//				//								pixel_difference_sum = 0;
			//				//								break;
			//				//							}
			//				//						}
			//				//						pixel_avg_difference = pixel_difference_sum/((i-r_edge+5)/5);
			//				//					}
			//				//
			//				//					now_5pixel_value = pixel[l_edge +10] + pixel[l_edge +9] + pixel[l_edge +8] + pixel[l_edge +7] + pixel[l_edge +6];
			//				//					for(int i = l_edge +5; i > 8; i = i-5){
			//				//						last_5pixel_value = now_5pixel_value;
			//				//						now_5pixel_value = pixel[i] + pixel[i-1] + pixel[i-2] + pixel[i-3] + pixel[i-4];
			//				//						pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5;
			//				//
			//				//						if(i == l_edge +5){
			//				//							pixel_avg_difference = pixel_difference_sum/((l_edge + 10 -i)/5);
			//				//							if(pixel_difference_sum >= 1500){
			//				//								pixel_avg_difference = 1;
			//				//							}
			//				//						}
			//				//						if(pixel_avg_difference >= 10){
			//				//							pixel_avg_difference = 1;
			//				//						}
			//				//
			//				//						if((now_5pixel_value - last_5pixel_value)/5 < -150*aabbss(pixel_avg_difference)){
			//				//							l_edge = i;
			//				//							l_color_flag = white_black;
			//				//							pixel_difference_sum = 0;
			//				//							break;
			//				//						}
			//				//
			//				//
			//				//						else if((now_5pixel_value - last_5pixel_value)/5 > 150*aabbss(pixel_avg_difference)){
			//				//							l_edge = i;
			//				//							l_color_flag = black_white;
			//				//							pixel_difference_sum = 0;
			//				//							break;
			//				//						}
			//				//						else if(i == 9){
			//				//
			//				//							if((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 > 60000){
			//				//								l_color_flag = half_white;
			//				//								pixel_difference_sum = 0;
			//				//								break;
			//				//							}
			//				//							else if((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 < 35000){
			//				//								l_color_flag = half_black;
			//				//								pixel_difference_sum = 0;
			//				//								break;
			//				//							}
			//				//							break;
			//				//						}
			//				//						pixel_avg_difference = pixel_difference_sum/((l_edge + 10 -i)/5);
			//				//					}
			//				//
			//				//					if(center_line_flag == 0){
			//				//						road_length = (r_edge - l_edge);
			//				//						center_line = (road_length)/2+l_edge;
			//				//						center_line_flag++;
			//				//					}
			//				//
			//				//					last_center_line_error = now_center_line_error;
			//				//					now_center_line_error = (float)(center_line - ((r_edge - l_edge)/2+l_edge));         //if the error is positive, car need to turn right
			//				//					center_line_error_change = now_center_line_error - last_center_line_error;
			//				//					center_line_errorsum += now_center_line_error;
			//				//					if(center_line_errorsum > 1000){
			//				//						center_line_errorsum = 1000;
			//				//					}
			//				//					else if(center_line_errorsum < -1000){
			//				//						center_line_errorsum = -1000;
			//				//					}
			//				//
			//				//
			//				//					turning_count += (int32_t)(now_center_line_error*turning_Kp + center_line_error_change*turning_Kd + center_line_errorsum*turning_Ki);
			//				//				}
			//
			//				//				else{
			//				now_5pixel_value = pixel[57] + pixel[58] + pixel[59] + pixel[60] + pixel[61];
			//				for(int i=62; i < 118; i = i+5){
			//					last_5pixel_value = now_5pixel_value;
			//					now_5pixel_value = pixel[i] + pixel[i+1] + pixel[i+2] + pixel[i+3] + pixel[i+4];
			//					pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5.0f;
			//
			//					if(i == 62){
			//						pixel_avg_difference = pixel_difference_sum/((i-57)/5.0f);
			//						if(pixel_difference_sum >= 6){
			//							pixel_avg_difference = 1;
			//						}
			//
			//					}
			//
			//					if(pixel_avg_difference <= 1){
			//						pixel_avg_difference = 1;
			//					}
			//					if((now_5pixel_value - last_5pixel_value)/5.0f < (float)(-20*aabbss(pixel_avg_difference))){
			//						r_edge = i;
			//						r_color_flag = white_black;
			//						pixel_difference_sum = 0;
			//						break;
			//					}
			//
			//
			//					else if((now_5pixel_value - last_5pixel_value)/5.0f > (float)(20*aabbss(pixel_avg_difference))){
			//						r_edge = i;
			//						r_color_flag = black_white;
			//						pixel_difference_sum = 0;
			//						break;
			//					}
			//					else if(i == 117){
			//						if((int)((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6) > 230){
			//							r_edge = 0;
			//							r_color_flag = half_white;
			//							pixel_difference_sum = 0;
			//							break;
			//						}
			//						else if((int)((pixel[65]+pixel[75]+pixel[85]+pixel[95]+pixel[110]+pixel[122])/6 )< 100){
			//							r_edge = 62;
			//							r_color_flag = half_black;
			//							pixel_difference_sum = 0;
			//							break;
			//						}
			//					}
			//					pixel_avg_difference = pixel_difference_sum/((i-57)/5.0f);
			//				}
			//
			//				now_5pixel_value = pixel[65] + pixel[66] + pixel[67] + pixel[68] + pixel[69];
			//				for(int i = 69; i > 8; i = i-5){
			//					last_5pixel_value = now_5pixel_value;
			//					now_5pixel_value = pixel[i] + pixel[i-1] + pixel[i-2] + pixel[i-3] + pixel[i-4];
			//					pixel_difference_sum += (now_5pixel_value - last_5pixel_value)/5.0f;
			//
			//					if(i == 69){
			//						pixel_avg_difference = pixel_difference_sum/((74-i)/5.0f);
			//						if(pixel_difference_sum >= 6){
			//							pixel_avg_difference = 1;
			//						}
			//					}
			//					if(pixel_avg_difference <= 1){
			//						pixel_avg_difference = 1;
			//					}
			//
			//					if((now_5pixel_value - last_5pixel_value)/5.0f < (float)(-20*aabbss(pixel_avg_difference))){
			//						l_edge = i;
			//						l_color_flag = white_black;
			//						pixel_difference_sum = 0;
			//						break;
			//					}
			//
			//
			//					else if((now_5pixel_value - last_5pixel_value)/5.0f > (float)(20*aabbss(pixel_avg_difference))){
			//						l_edge = i;
			//						l_color_flag = black_white;
			//						pixel_difference_sum = 0;
			//						break;
			//					}
			//					else if(i == 9){
			//
			//						if((int)((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6) > 230){
			//							l_edge = 0;
			//							l_color_flag = half_white;
			//							pixel_difference_sum = 0;
			//							break;
			//						}
			//						else if((int)((pixel[64]+pixel[55]+pixel[45]+pixel[35]+pixel[20]+pixel[5])/6 )< 100){
			//							l_edge = 69;
			//							l_color_flag = half_black;
			//							pixel_difference_sum = 0;
			//							break;
			//						}
			//						break;
			//					}
			//					pixel_avg_difference = pixel_difference_sum/((74 - i)/5.0f);
			//				}
			//
			//				if(center_line_flag == 0 && l_edge > 0 && r_edge >0){
			//					road_length = (int16_t)(r_edge - l_edge);
			//					center_line = (int16_t)((road_length)/2.0f)+l_edge;
			//					center_line_flag++;
			//				}
			//
			//				last_center_line_error = now_center_line_error;
			//				now_center_line_error = (float)(center_line - ((r_edge - l_edge)/2+l_edge));         //if the error is positive, car need to turn right
			//				center_line_error_change = now_center_line_error - last_center_line_error;
			//				center_line_errorsum += now_center_line_error;
			//				if(center_line_errorsum > 1000){
			//					center_line_errorsum = 1000;
			//				}
			//				else if(center_line_errorsum < -1000){
			//					center_line_errorsum = -1000;
			//				}
			//
			//
			//				turning_count += (int32_t)(now_center_line_error*turning_Kp + center_line_error_change*turning_Kd + center_line_errorsum*turning_Ki);
			//				//				}
			//
			//
			//
			//				//								if(now_center_line_error >=0){
			//				//									turn[0] = ccd_Kp*now_center_line_error + ccd_Kd*center_line_error_change;
			//				//									turn[1] =
			//				//								}
			//				//				lincoln.Set(0);
			//				//				pt6 = System::Time();
			//			}






			//				else if(yo == 5){
			//					yo = 0;
			//				}

			//			//						if((int32_t)(t-pt7) >= 100){
			//			if(t%100 == 0){
			//
			//				libsc::St7735r::Rect rect_;
			//
			//				for(int i=0;i<=128;i++){
			//					rect_.x = i;
			//					rect_.y = 160-(int)(old_pixel[i]*30/255.0f);
			//					rect_.w = 1;
			//					rect_.h = 1;
			//					lcd.SetRegion(rect_);
			//					lcd.FillColor(BLACK);
			//
			//
			//				}
			//
			//
			//
			//
			//
			//
			//				for(int i=0;i<=128;i++){
			//					rect_.x = i;
			//					rect_.y = 160-(int)(pixel[i]*30/255.0f);
			//					rect_.w = 1;
			//					rect_.h = 1;
			//					lcd.SetRegion(rect_);
			//					lcd.FillColor(GREEN);
			//				}
			//
			//				//				for(int i=0; i<127;i++){
			//				//
			//				//					if(pixel[i+1]>pixel[i] && i>=0){
			//				//						memory=pixel[i+1];
			//				//					}
			//				//				}
			//				console.SetTextColor(GREEN);
			//				console.SetCursorRow(0);
			//				sprintf(buffer, "l_edge:%d\nr_edge:%d\nCenterline:%d\nError:%.3f\nl_encoder:%d\nr_encoder:%d\n",l_edge,r_edge,center_line,now_center_line_error,il_encoder_error,ir_encoder_error);
			//				console.WriteString((char*)buffer);
			//
			//
			//
			//
			//
			//
			//
			//				old_pixel=pixel;
			//
			//				//				if(l_color_flag == white_black){
			//				//					if(r_color_flag == white_black){
			//				//						//reset the flag
			//				//
			//				//						rect_.x = 0;
			//				//						rect_.y = 130;
			//				//						rect_.w = l_edge;
			//				//						rect_.h = 16;
			//				//						lcd.SetRegion(rect_);
			//				//						lcd.FillColor(BLACK);
			//				//
			//				//
			//				//						rect_.x = l_edge;
			//				//						rect_.y = 130;
			//				//						rect_.w = r_edge - l_edge;
			//				//						rect_.h = 16;
			//				//						lcd.SetRegion(rect_);
			//				//						lcd.FillColor(WHITE);
			//				//
			//				//
			//				//						rect_.x = r_edge;
			//				//						rect_.y = 130;
			//				//						rect_.w = 128 - r_edge;
			//				//						rect_.h = 16;
			//				//						lcd.SetRegion(rect_);
			//				//						lcd.FillColor(BLACK);
			//				//
			//				//
			//				//
			//				//					}
			//				//					else if(r_color_flag == half_white){
			//				//
			//				//
			//				//						rect_.x = 0;
			//				//						rect_.y = 130;
			//				//						rect_.w = l_edge;
			//				//						rect_.h = 16;
			//				//						lcd.SetRegion(rect_);
			//				//						lcd.FillColor(BLACK);
			//				//						lcd_flag = 1;
			//				//
			//				//
			//				//						rect_.x = l_edge;
			//				//						rect_.y = 130;
			//				//						rect_.w = 128 - l_edge;
			//				//						rect_.h = 16;
			//				//						lcd.SetRegion(rect_);
			//				//						lcd.FillColor(WHITE);
			//				//						lcd_flag = 0;
			//				//
			//				//					}
			//				//				}
			//				//				else if(l_color_flag == half_white){
			//				//					if(r_color_flag == white_black){
			//				//						rect_.x = 0;
			//				//						rect_.y = 130;
			//				//						rect_.w = r_edge;
			//				//						rect_.h = 16;
			//				//						lcd.SetRegion(rect_);
			//				//						lcd.FillColor(WHITE);
			//				//
			//				//						rect_.x = r_edge;
			//				//						rect_.y = 130;
			//				//						rect_.w = 128 - r_edge;
			//				//						rect_.h = 16;
			//				//						lcd.SetRegion(rect_);
			//				//						lcd.FillColor(BLACK);
			//				//					}
			//				//					else if(r_color_flag == half_white){
			//				//						rect_.x = 0;
			//				//						rect_.y = 130;
			//				//						rect_.w = 128;
			//				//						rect_.h = 16;
			//				//						lcd.SetRegion(rect_);
			//				//						lcd.FillColor(WHITE);
			//				//					}
			//				//				}
			//				//				pt7 = System::Time();
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
