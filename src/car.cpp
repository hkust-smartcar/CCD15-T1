///*
// * car.cpp
// *
// *  Created on: 9 Mar, 2015
// *      Author: sfuab
// */
//
//#include<car.h>
//
//
//#include <car.h>
//
//using namespace libbase;
//using namespace libsc;
//using namespace libbase::k60;
//using namespace libsc::k60;
//using namespace libutil;
//
//namespace libbase
//{
//namespace k60
//{
//
//Mcg::Config Mcg::GetMcgConfig()
//{
//	Mcg::Config config;
//	config.external_oscillator_khz = 50000;
//	config.core_clock_khz = 150000;
//	return config;
//}
//
//}
//}
//
//JyMcuBt106::Config Get_Bluetooth_Config()
//{
//	JyMcuBt106::Config bt_config;
//	bt_config.id = 0;
//	bt_config.rx_irq_threshold = 2;
//	bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//	return bt_config;
//}
//
//Mpu6050::Config Get_Gyro_Config()
//{
//	Mpu6050::Config gyro_config;
//	//sensitivity of gyro
//	gyro_config.gyro_range = Mpu6050::Config::Range::kLarge;
//	//sensitivity of accelerometer
//	gyro_config.accel_range = Mpu6050::Config::Range::kLarge;
//	return gyro_config;
//}
//
//Joystick::Config Get_Joystick_Config()
//{
//	Joystick::Config joycon;
//	joycon.id = 0;
//	joycon.is_active_low = true;
//	return joycon;
//}
//
//AlternateMotor::Config Get_Motor_l_Config()
//{
//	AlternateMotor::Config l_motor;
//	l_motor.id = 1;
//	return l_motor;
//}
//
//AlternateMotor::Config Get_Motor_r_Config()
//{
//	AlternateMotor::Config r_motor;
//	r_motor.id = 1;
//	return r_motor;
//}
//
//DirEncoder::Config Get_Encoder_l_Config()
//{
//	DirEncoder::Config l_encoder;
//	l_encoder.id = 0;
//	return l_encoder;
//}
//
//DirEncoder::Config Get_Encoder_r_Config()
//{
//	DirEncoder::Config r_encoder;
//	r_encoder.id = 0;
//	return r_encoder;
//}
//
//Adc::Config Get_Adc_Config()
//{
//	Adc::Config adc_config;
//	adc_config.pin = Pin::Name::kPta17;
//	return adc_config;
//}
//
//
//Car::Car():
//		trust_accel(0.01), trust_old_accel(0)
//{
//	bt(Get_Bluetooth_Config());
//	mpu6050(Get_Gyro_Config());
//	joy(Get_Joystick_Config());
//	motor_l(Get_Motor_l_Config());
//	motor_r(Get_Motor_r_Config());
//	encoder_l(Get_Encoder_l_Config());
//	encoder_r(Get_Encoder_r_Config());
//	ccd(0);
//	PowerTest(Get_Adc_Config());
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
