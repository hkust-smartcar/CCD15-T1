/*
 * car.h
 *
 *  Created on: 3 Mar, 2015
 *      Author: Howard
 */
#pragma once

#include <libbase/helper.h>
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
#include "kalman.h"
#include <libsc/button.h>
#include <libsc/simple_buzzer.h>
#include "upstand.h"
#include "Adverage.h"
#include <cmath>
#include <libbase/k60/flash.h>
#include "MyFlash.h"
#include "MyVar.h"
#include "pVarManager.h"

//using namespace libsc;
//using namespace libsc::k60;
//using namespace libutil;
using namespace LIBBASE_NS;
using namespace libsc;
using namespace LIBSC_NS;
using namespace LIBSC_NS;
using namespace libutil;



class Car{
public:
	Car();
//	RemoteVarManager* varmanager;
//	JyMcuBt106 *m_bt;
	Mpu6050 *m_mpu6050;
	Mma8451q *m_mma8451q;
	Joystick *m_joy;
//	DirMotor *motor_l, *motor_r;
	AlternateMotor *motor_l, *motor_r;
	AbEncoder *encoder_l, *encoder_r;
//	libbase::k60::Adc *PowerTest, *angle_tuner;
	St7735r *m_lcd;
	LcdTypewriter *m_lcd_console;
	Button *m_button0, *m_button1;
	libsc::Led *m_led0, *m_led1, *m_led2;
	Tsl1401cl *ccd_down, *ccd_up;
	SimpleBuzzer *m_buzzer;
	Upstand* m_upstand;

	Kalman *speedKF;
	Kalman *turnKF;

	Byte border_state;
	int car_bt_print;
	float car_raw_angle;

//	RemoteVarManager::Var* is_Kp;
//	RemoteVarManager::Var* is_Kd;
//	RemoteVarManager::Var* ideal_speed;
//	RemoteVarManager::Var* ic_Kp;
//	RemoteVarManager::Var* ic_Kd;

//	float output_angle,original_angle,accel_angle,gyro_angle,ic_Kd,ic_Kp,ic_Ki,trust_accel,ideal_speed,is_Kp,is_Kd;

	void SelectLeft(const uint8_t)
	{
		car_bt_print -= 1;
		if(car_bt_print < 0)
			car_bt_print = 3;
	}

	void SelectRight(const uint8_t)
	{
		car_bt_print += 1;
		if(car_bt_print > 3)
			car_bt_print = 0;
	}

	void SelectUp(const uint8_t)
	{
		car_raw_angle -= 0.5;
	}

	void SelectDown(const uint8_t)
	{
		car_raw_angle += 0.5;
	}

	void Select(const uint8_t)
	{
		border_state = 1;
	}

	Joystick::Config Get_Joystick_Config()
	{
		Joystick::Config joycon;
		joycon.id = 0;
		joycon.is_active_low = true;
		joycon.listener_triggers[(uint8_t)Joystick::State::kUp] = Joystick::Config::Trigger::kDown;
		joycon.listeners[(uint8_t)Joystick::State::kUp] =  std::bind(&Car::SelectUp, this, std::placeholders::_1);
		joycon.listener_triggers[(uint8_t)Joystick::State::kDown] = Joystick::Config::Trigger::kDown;
		joycon.listeners[(uint8_t)Joystick::State::kDown] =  std::bind(&Car::SelectDown, this, std::placeholders::_1);
		joycon.listener_triggers[(uint8_t)Joystick::State::kLeft] = Joystick::Config::Trigger::kDown;
		joycon.listeners[(uint8_t)Joystick::State::kLeft] =  std::bind(&Car::SelectLeft, this, std::placeholders::_1);
		joycon.listener_triggers[(uint8_t)Joystick::State::kRight] = Joystick::Config::Trigger::kDown;
		joycon.listeners[(uint8_t)Joystick::State::kRight] =  std::bind(&Car::SelectRight, this, std::placeholders::_1);
		joycon.listener_triggers[(uint8_t)Joystick::State::kSelect] = Joystick::Config::Trigger::kDown;
		joycon.listeners[(uint8_t)Joystick::State::kSelect] =  std::bind(&Car::Select, this, std::placeholders::_1);
		return joycon;
	}
private:
};

