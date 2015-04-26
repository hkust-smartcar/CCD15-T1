///*
// * car.cpp
// *
// *  Created on: 9 Mar, 2015
// *      Author: sfuab
// */
//#include <car.h>
//
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
//
//Joystick::Config Get_Joystick_Config()
//{
//	Joystick::Config joycon;
//	joycon.id = 0;
//	joycon.is_active_low = true;
//	return joycon;
//}
//
//
//AlternateMotor::Config Get_Motor_L_Config()
//{
//	AlternateMotor::Config l_motor;
//	l_motor.id = 0;
//	return l_motor;
//}
//
//
//AlternateMotor::Config Get_Motor_R_Config()
//{
//	AlternateMotor::Config r_motor;
//	r_motor.id = 1;
//	return r_motor;
//}
//
//
//AbEncoder::Config Get_Encoder_L_Config()
//{
//	AbEncoder::Config l_encoder;
//	l_encoder.id = 0;
//	return l_encoder;
//}
//
//
//AbEncoder::Config Get_Encoder_R_Config()
//{
//	AbEncoder::Config r_encoder;
//	r_encoder.id = 0;
//	return r_encoder;
//}
//
//
//Adc::Config Get_PowerTest_Config()
//{
//	Adc::Config adc_config;
//	adc_config.pin = Pin::Name::kPta17;
//	return adc_config;
//}
//
//St7735r::Config Get_Lcd_Config()
//{
//	St7735r::Config lcd_config;
//	lcd_config.is_revert = false;
//	return lcd_config;
//}
//
//Button::Config Get_Button0_Config()
//{
//    Button::Config b_config0;
//    b_config0.id = 0;
//    b_config0.is_active_low = true;
//    return b_config0;
//}
//
//Button::Config Get_Button1_Config()
//{
//    Button::Config b_config1;
//    b_config1.id = 0;
//    b_config1.is_active_low = true;
//    return b_config1;
//}
//
//libsc::Led::Config Get_LED0_Config()
//{
//	libsc::Led::Config led_config0;
//    led_config0.id = 0;
//    return led_config0;
//}
//
//libsc::Led::Config Get_LED1_Config()
//{
//	libsc::Led::Config led_config1;
//    led_config1.id = 0;
//    return led_config1;
//}
//
//
//Car::Car():
//		trust_accel(0.01), trust_old_accel(0)
//{
//	bt(Get_Bluetooth_Config());
//	mpu6050(Get_Gyro_Config());
//	joy(Get_Joystick_Config());
//	motor_l(Get_Motor_L_Config());
//	motor_r(Get_Motor_R_Config());
//	encoder_l(Get_Encoder_L_Config());
//	encoder_r(Get_Encoder_R_Config());
//	Tsl1401cl ccd(0);
//	PowerTest(Get_PowerTest_Config());
//	lcd(Get_Lcd_Config());
//	button0(Get_Button0_Config());
//	button1(Get_Button1_Config());
//	led0(Get_LED0_Config());
//	led1(Get_LED1_Config());
//	acc(0.001f, kalman_value, 0, 1);
//
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
