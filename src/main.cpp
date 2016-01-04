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
//#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/ab_encoder.h>
#include <libutil/remote_var_manager.h>
#include "kalman.h"
#include <libsc/button.h>
#include <libsc/simple_buzzer.h>
#include "upstand.h"
#include "app.h"
#include "car.h"
#include "MyVarManager.h"

int main()
{
	//intialize the system
	System::Init();
	Timer::TimerInt t = 0, pt[5] = {0, 0, 0, 0, 0};
	Byte yo = 0;
	Byte ccd_id = 0;

	App m_app;
	m_app.RawAngle();
//	m_app.pgrapher_setup();
	//	//graph testing variable
	//
//	MyVarManager pGrapher;
		//	//graph testing variable
		//
	float ha = 10;
	MyVarManager pGrapher;

		pGrapher.addWatchedVar(&ha,"ha");




	while(1){
		t = System::Time();

		if((int32_t)(t-pt[0]) >= 7  && yo == 0){
			pt[0] = System::Time();
			yo = 1;

			m_app.Common(1);
			m_app.Balance_PID();
		}

		if((int32_t)(t-pt[0]) >= 1 && yo == 1){
			pt[1]=System::Time();
			yo = 2;
			m_app.Common(1);
			m_app.Speed_PID();
			m_app.SetMotors();
		}



		if((int32_t)(t-pt[1]) >= 1 && yo == 2){
			pt[2] = System::Time();
			yo = 3;
			m_app.Common(1);

			// For the top CCD to prediction
			if(ccd_id == 1)
			{
				m_app.CcdGetValue(ccd_id, m_app.m_car.ccd_up);
				m_app.UpCcd();
				ccd_id = 0;
			}

			/*  The lower CCD's turn.
			 *  It decides the middle line.
			 *  And cancels states.
			 *
			 *
			 */
			else if(ccd_id == 0)
			{
				m_app.CcdGetValue(ccd_id, m_app.m_car.ccd_down);
				m_app.DownCcd();
				ccd_id = 1;
			}

			m_app.Turn_PID();

		}



		/*second round to get angle and encoder
		 *
		 *
		 *
		 *
		 */
		if((int32_t)(t-pt[2]) >= 2  && yo == 3){
			pt[3] = System::Time();
			yo = 4;

			m_app.Common(2);
			m_app.Balance_PID();
		}


		if((int32_t)(t-pt[3]) >= 1 && yo == 4){
			pt[4] = System::Time();
			yo = 5;

			m_app.Common(1);
			m_app.Speed_PID();
			m_app.SetMotors();
		}


		if((int32_t)(t-pt[4]) >= 1 && yo == 5){
			yo =0;

			m_app.Common(1);
			pGrapher.sendWatchData();
//			m_app.BluetoothSend();
		}


	}
}



