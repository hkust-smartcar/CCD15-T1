/*
 * car.cpp
 *
 *  Created on: 9 Mar, 2015
 *      Author: sfuab
 */
#include <car.h>


#include<car.h>


#include <car.h>

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
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 180000;
	return config;
}

}
}


//JyMcuBt106::Config Get_Bluetooth_Config(RemoteVarManager* varmanager)
//{
//	JyMcuBt106::Config bt_config;
//	bt_config.id = 0;
//	bt_config.rx_irq_threshold = 2;
//	bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//	bt_config.rx_isr = std::bind(&RemoteVarManager::OnUartReceiveChar, varmanager, std::placeholders::_1,
//				std::placeholders::_2);
//	return bt_config;
//}


Mpu6050::Config Get_Gyro_Config()
{
	Mpu6050::Config gyro_config;
	//sensitivity of gyro
	gyro_config.gyro_range = Mpu6050::Config::Range::kSmall;
	//sensitivity of accelerometer
	gyro_config.accel_range = Mpu6050::Config::Range::kSmall;
	gyro_config.cal_drift = true;
	return gyro_config;
}

Mma8451q::Config Get_Accel_Config(Mpu6050 *m_mpu6050)
{
	Mma8451q::Config accel_config;
	accel_config.id = 0;
	accel_config.power_mode = Mma8451q::Config::PowerMode::kLowNoiseLowPower;
	accel_config.output_data_rate = Mma8451q::Config::OutputDataRate::k200Hz;
	//	accel_config.i2c_master_ptr = m_mpu6050->GetI2cMaster();
	return accel_config;
}



AlternateMotor::Config Get_Motor_L_Config()
{
	AlternateMotor::Config l_motor;
	l_motor.id = 1;
	return l_motor;
}
AlternateMotor::Config Get_Motor_R_Config()
{
	AlternateMotor::Config r_motor;
	r_motor.id = 0;
	return r_motor;
}


AbEncoder::Config Get_Encoder_L_Config()
{
	AbEncoder::Config l_encoder;
	l_encoder.id = 0;
	return l_encoder;
}
AbEncoder::Config Get_Encoder_R_Config()
{
	AbEncoder::Config r_encoder;
	r_encoder.id = 1;
	return r_encoder;
}


//libbase::k60::Adc::Config Get_PowerTest_Config()
//{
//	libbase::k60::Adc::Config adc_config;
//	adc_config.pin = libbase::k60::Pin::Name::kPtb8;
//	return adc_config;
//}
//libbase::k60::Adc::Config Get_Angle_Tuner_Config()
//{
//	libbase::k60::Adc::Config adc_config;
//	adc_config.pin = libbase::k60::Pin::Name::kPtd1;
//	return adc_config;
//}

St7735r::Config Get_Lcd_Config()
{
	St7735r::Config lcd_config;
	lcd_config.is_revert = false;
	return lcd_config;
}

LcdTypewriter::Config Get_Lcd_console_Config(St7735r *lcd){
	LcdTypewriter::Config lcd_config;
	lcd_config.lcd = lcd;
	lcd_config.is_text_wrap = true;
	return lcd_config;
}

Button::Config Get_Button0_Config()
{
	Button::Config b_config0;
	b_config0.id = 0;
	b_config0.is_active_low = true;
	return b_config0;
}
Button::Config Get_Button1_Config()
{
	Button::Config b_config1;
	b_config1.id = 1;
	b_config1.is_active_low = true;
	return b_config1;
}


libsc::Led::Config Get_LED0_Config()
{
	libsc::Led::Config led_config0;
	led_config0.id = 0;
	return led_config0;
}
libsc::Led::Config Get_LED1_Config()
{
	libsc::Led::Config led_config1;
	led_config1.id = 1;
	return led_config1;
}
libsc::Led::Config Get_LED2_Config()
{
	libsc::Led::Config led_config2;
	led_config2.id = 2;
	return led_config2;
}

SimpleBuzzer::Config Get_Buzzer_Config()
{
	SimpleBuzzer::Config buzzer_config;
	buzzer_config.id = 0;
	buzzer_config.is_active_low = 1;
	return buzzer_config;
}




Car::Car():
										border_state(0), car_bt_print(0), car_raw_angle(0)
{
	//	varmanager = new RemoteVarManager(5);
	//		m_bt = new JyMcuBt106(Get_Bluetooth_Config(varmanager));
	m_mpu6050 = new Mpu6050(Get_Gyro_Config());
	m_mma8451q = new Mma8451q(Get_Accel_Config(m_mpu6050));
	//	m_joy = new Joystick(Get_Joystick_Config());
	//	motor_l = new DirMotor(Get_Motor_L_Config());
	//	motor_r = new DirMotor(Get_Motor_R_Config());
	motor_l = new AlternateMotor(Get_Motor_L_Config());
	motor_r = new AlternateMotor(Get_Motor_R_Config());
	encoder_l = new AbEncoder(Get_Encoder_L_Config());
	encoder_r = new AbEncoder(Get_Encoder_R_Config());
	//	PowerTest = new libbase::k60::Adc(Get_PowerTest_Config());
	//	angle_tuner = new libbase::k60::Adc(Get_Angle_Tuner_Config());
	m_lcd = new St7735r(Get_Lcd_Config());
	//	m_lcd_console = new LcdTypewriter(Get_Lcd_console_Config(m_lcd));
	m_button0 = new Button(Get_Button0_Config());
	m_button1 = new Button(Get_Button1_Config());
	//	m_led0 = new libsc::Led(Get_LED0_Config());
	//	m_led1 = new libsc::Led(Get_LED1_Config());
	//	m_led2 = new libsc::Led(Get_LED2_Config());
	ccd_down = new Tsl1401cl(0);
	ccd_up = new Tsl1401cl(1);
	m_buzzer = new SimpleBuzzer(Get_Buzzer_Config());
	m_upstand = new Upstand((m_mpu6050), (m_mma8451q));

	m_buzzer->SetBeep(1);
	motor_l->SetPower(0);
	motor_r->SetPower(0);

	//	libutil::InitDefaultFwriteHandler(m_bt);

	double _R[2] = {0.05, -1};
	speedKF = new Kalman(0.000007, _R, 0.0, 1.0);
	_R[0] = 0.01;
	turnKF = new Kalman(0.00028, _R,0.0,1.0);

	//	is_Kp = varmanager->Register("is_Kp",RemoteVarManager::Var::Type::kReal);
	//	is_Kd = varmanager->Register("is_Kd",RemoteVarManager::Var::Type::kReal);
	//	ideal_speed = varmanager->Register("ideal_speed",RemoteVarManager::Var::Type::kReal);
	//	ic_Kp = varmanager->Register("ic_Kp",RemoteVarManager::Var::Type::kReal);
	//	ic_Kd = varmanager->Register("ic_Kd",RemoteVarManager::Var::Type::kReal);
	//	varmanager->Broadcast(m_bt);

}





































