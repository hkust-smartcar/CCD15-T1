///*
// * app.cpp
// *
// *  Created on: 9 Mar, 2015
// *      Author: sfuab
// */
//
//#include <app.h>
//
//
//#include <app.h>
//
//
//#include <app.h>
//
//float App::GetOriginalAngle(){
//	t = System::Time();
//	pt = t;
//	while(1){
//		t = System::Time();
//		m_car.mpu6050.Update();
//		accel = m_car.mpu6050.GetAccel();
//		original_angle = accel[0]*57.29578;
//		if(t-pt <0)
//			pt=0;
//		if((t-pt)>=2000)
//			return original_angle;
//	}
//
//}
//
//
//
//void App::GetAngles()
//{
//	m_car.mpu6050.Update();
//
//	accel = m_car.mpu6050.GetAccel();
//	omega = m_car.mpu6050.GetOmega();
//
//	float last_accel_angle = accel_angle;
//	accel_angle = accel[0]*57.29578;
//	accel_angle = m_car.trust_old_accel * last_accel_angle + (1 - trust_old_accel) * accel_angle;
//
//	float last_gyro_angle = gyro_angle;
//	gyro_angle += (-1) * omega[1] * 0.005 + m_car.trust_accel * (accel_angle - gyro_angle);
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
//int App::DeadZoneTesting(int &dead_value_l, int &dead_value_r)
//{
//	while(1){
//		m_car.motor_l.SetClockwise(0);
//		m_car.motor_r.SetClockwise(0);
//		m_car.motor_l.SetPower(dead_value_l);
//		m_car.motor_r.SetPower(dead_value_r);
//		m_car.encoder_l.Update();
//		m_car.encoder_r.Update();
//		System::DelayMs(10);
//		m_car.encoder_l.Update();
//		m_car.encoder_r.Update();
//		if(m_car.encoder_l.GetCount())
//			dead_value_l -= 3;
//		if(m_car.encoder_r.GetCount())
//			dead_value_r -= 3;
//		if(m_car.encoder_l.GetCount() == 0 && m_car.encoder_r.GetCount() ==0)
//		{
//			while(1){
//				m_car.motor_l.SetPower(dead_value_l);
//				m_car.motor_r.SetPower(dead_value_r);
//				m_car.encoder_l.Update();
//				m_car.encoder_r.Update();
//				System::DelayMs(10);
//				m_car.encoder_l.Update();
//				m_car.encoder_r.Update();
//				count_l = m_car.encoder_l.GetCount();
//				count_r = -m_car.encoder_r.GetCount();
//				if(count_l <= 0)
//					dead_value_l += 3;
//				if(count_r <= 0)
//					dead_value_r += 3;
//				if(count_l > 0 && count_l > 0)
//				{
//					m_car.motor_l.SetPower(0);
//					m_car.motor_r.SetPower(0);
//					count_l = 0;
//					count_r = 0;
//					return 0;
//				}
//			}
//		}
//	}
//}
//
//
//
//
//void App::GetIdealCount()
//{
//	angle_error[0] = original_angle - output_angle;
//	angle_error[2] =  angle_error[0] - angle_error[1];      // angle_error_change = angle_error[0] -last_angle_error;
//
//	int angle_gain = 1;
//	if(angle_error[0] > 4 || angle_error[0] < -4)
//		angle_gain = 2;
//	else if(angle_error[0] > 7 || angle_error[0] < -6)
//		angle_gain = 25;
//	else if(angle_error[0] > 10 || angle_error[0] < -8)
//		angle_gain = 200;
//	else if(angle_error[0] > 16 || angle_error[0] < -14)
//		angle_gain = 400;
//
//	ideal_count[1] = ideal_count[0];
//	ideal_count[0] = (int32_t)(ic_Kp * angle_error[0] * angle_gain + angle_gain * ic_Kd * angle_error[2] / 0.003 - still_Ki * total_count_r);
//	ideal_count[0] = (int32_t)(0.2*ideal_count[1] + 0.8*ideal_count[0]);
//
//	angle_error[1] = angle_error[0];
//}
//
//void App::Balance_PID()
//{
//	GetAngles();
//	GetOutputAngle();
//	GetIdealCount();
//}
//
//void App::GetEncoderValues()
//{
//	m_car.encoder_r.Update();
//	m_car.encoder_l.Update();
//
//	count_r = (int32_t)(-m_car.encoder_r.GetCount());
//	count_l = (int32_t)(m_car.encoder_l.GetCount());
//
//	//	total_count_l += (float)count_l * 0.003;
//	//	total_count_r += (float)count_r * 0.003;
//
//	sign[1] = sign[0];
//	if(ideal_count[0] > 0)
//		sign[0] = 0;
//	else if(ideal_count[0] < 0)
//		sign[0] = 1;
//	else if(ideal_count[0] ==0)
//		sign[0] = 2;
//
//
//	if(sign[0] == 2){
//		m_car.motor_l.SetPower(0);
//		m_car.motor_r.SetPower(0);
//	}
//
//	else{
//
//		if(sign[1] != sign[0]){
//			m_car.motor_l.SetClockwise(sign);
//			m_car.motor_r.SetClockwise(sign);
//		}
//
//		encoder_error[3] = encoder_error[2];               //last_encoder_error[2] = encoder_error[2];
//		encoder_error[2] = ideal_count[0] - count_r;
//		encoder_error[1] = encoder_error[0];	           //last_encoder_error[0] = encoder_error[0];
//		encoder_error[0] = ideal_count[0] - count_l;
//
//
//		encoder_error_change[0] = encoder_error[0] -encoder_error[1];      //il_encoder_error_change = il_encoder_error -last_il_encoder_error;
//		encoder_error_change[2] = encoder_error[2] -encoder_error[3];      //ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;
//
//		encoder_error_change[2] = 0.8f * encoder_error_change[2] + 0.2f * encoder_error_change[3];        // ir_encoder_error_change = 0.8 * ir_encoder_error_change + 0.2 * last_ir_encoder_error_change;
//		encoder_error_change[0] = 0.8f * encoder_error_change[0] + 0.2f * encoder_error_change[1];        // il_encoder_error_change = 0.8 * il_encoder_error_change + 0.2 * last_il_encoder_error_change;
//
//		encoder_error_change[1] = encoder_error_change[0];        // last_il_encoder_error_change = il_encoder_error -last_il_encoder_error;
//		encoder_error_change[3] = encoder_error_change[2];        // last_ir_encoder_error_change = ir_encoder_error -last_ir_encoder_error;
//	}
//}
//
//
//void App::SetSpeed()
//{
//	float encoder_gain = 1;
//	if(ideal_count > 400 || ideal_count < -400)
//		encoder_gain = 3.0f;
//	else if (ideal_count < 40 || ideal_count > -40)
//		encoder_gain = 0.2f;
//	else if(ideal_count > 600 || ideal_count < -600)
//		encoder_gain = 10.0f;
//
//	speed[1] = (int32_t)((float)(encoder_error[2] *encoder_r_Kp * encoder_gain) + encoder_gain * encoder_error_change[2] * encoder_r_Kd / 0.003f);
//	speed[0] = (int32_t)((float)(encoder_error[0] *encoder_l_Kp * encoder_gain) + encoder_gain * encoder_error_change[0] * encoder_l_Kd / 0.003f);
//
//	if(speed[0] > 900 && sign[0] == 0)
//		speed[0] = 900;
//	else if(speed[0] < 0 && sign[0] ==0)
//		speed[0] = 0;
//	else if(speed[0] > 0 && sign[0] ==1)
//		speed[0] = 0;
//	else if(speed[0] < -900 && sign[0] ==1)
//		speed[0] = -900;
//
//	if(speed[1] > 900 && sign[0] ==0)
//		speed[1] = 900;
//	else if(speed[1] < 0 && sign[0] ==0)
//		speed[1] = 0;
//	else if(speed[1] > 0 && sign[0] ==1)
//		speed[1] = 0;
//	else if(speed[1] < -900 && sign[0] ==1)
//		speed[1] = -900;
//
//	speed[1] = speed[1]*turn[1];
//	speed[0] = speed[0]*turn[0];
//
//	m_car.motor_l.SetPower(abs(speed[0])+dead_value_l);
//	m_car.motor_r.SetPower(abs(speed[1])+dead_value_r);
//}
//
//void App::Encoder_PID()
//{
//	GetEncoderValues();
//	SetSpeed();
//}
//
//void App::CcdGetValue()
//{
//	m_car.ccd.StartSample();
//	while(!m_car.ccd.SampleProcess()){}
//
//	Pixel = m_car.ccd.GetData();
//
//}
//
//void App::CcdGetAver()
//{
//	uint16_t ccd_max = Pixel[0], ccd_min = Pixel[0];
//	for(int i = 1; i < LinearCcd::kSensorW; i++)
//	{
//		if (Pixel[i] > ccd_max)
//			ccd_max = Pixel[i];
//	}
//	for(int i = 1; i < LinearCcd::kSensorW; i++)
//	{
//		if (Pixel[i] < ccd_min)
//			ccd_min = Pixel[i];
//	}
//
//	if (ccd_max - ccd_min >= ???)
//	ccd_aver = (ccd_max + ccd_min) / 2;
//	else
//	{
//		if(ccd_max > ???)
//			ccd_aver = min - ?;
//		else
//		    ccd_aver = max + ?;
//	}
//}
//
//
//
//
//
//
//
//
//int App::Tuner()
//{
//	char received;
//	while(1){
//		if(m_car.bt.PeekChar(&received))
//		{
//			switch(received)
//			{
//			case 'z':
//			ic_Kp += 0.1;
//			break;
//			case 'Z':
//				ic_Kp += 1;
//				break;
//			case 'x':
//				if(ic_Kp >= 0.1)
//					ic_Kp -= 0.1;
//				break;
//			case 'X':
//				if(ic_Kp >= 1)
//					ic_Kp -= 1;
//				break;
//			case 'c':
//				ic_Kd += 0.001;
//				break;
//			case 'C':
//				ic_Kd += 0.01;
//				break;
//			case 'v':
//				if(ic_Kd >= 0.001)
//					ic_Kd -= 0.001;
//				break;
//			case 'V':
//				if(ic_Kd >= 0.01)
//					ic_Kd -= 0.01;
//				break;
//
//			case 'b':
//				ic_Ki += 0.1;
//				break;
//			case 'B':
//				ic_Ki += 1;
//				break;
//			case 'n':
//				if(ic_Ki >= 0.1)
//					ic_Ki -= 0.1;
//				break;
//			case 'N':
//				if(ic_Ki >= 1)
//					ic_Ki -= 1;
//				break;
//
//
//			case 'q':
//				encoder_l_Kp += 0.2;
//				break;
//			case 'Q':
//				encoder_l_Kp += 1;
//				break;
//			case 'w':
//				if(encoder_l_Kp >= 0.2)
//					encoder_l_Kp -= 0.2;
//				break;
//			case 'W':
//				if(encoder_l_Kp >= 1)
//					encoder_l_Kp -= 1;
//				break;
//
//			case 'e':
//				encoder_l_Kd += 0.001;
//				break;
//			case 'E':
//				encoder_l_Kd += 0.01;
//				break;
//			case 'r':
//				if(encoder_l_Kd >= 0.001)
//					encoder_l_Kd -= 0.001;
//				break;
//			case 'R':
//				if(encoder_l_Kd >= 0.01)
//					encoder_l_Kd -= 0.01;
//				break;
//
//			case 't':
//				encoder_l_Ki += 0.001;
//				break;
//			case 'T':
//				encoder_l_Ki += 0.1;
//				break;
//			case 'y':
//				if(encoder_l_Ki >= 0.001)
//					encoder_l_Ki -= 0.001;
//				break;
//			case 'Y':
//				if(encoder_l_Ki >= 0.1)
//					encoder_l_Ki -= 0.1;
//				break;
//
//			case 'a':
//				encoder_r_Kp += 1;
//				break;
//			case 'A':
//				encoder_r_Kp += 100;
//				break;
//			case 's':
//				if(encoder_r_Kp >= 1)
//					encoder_r_Kp -= 1;
//				break;
//			case 'S':
//				if(encoder_r_Kp >= 100)
//					encoder_r_Kp -= 100;
//				break;
//
//			case 'd':
//				encoder_r_Kd += 0.1;
//				break;
//			case 'D':
//				encoder_r_Kd += 1;
//				break;
//			case 'f':
//				if(encoder_r_Kd >= 0.1)
//					encoder_r_Kd -= 0.1;
//				break;
//			case 'F':
//				if(encoder_r_Kd >= 1)
//					encoder_r_Kd -= 1;
//				break;
//
//			case 'g':
//				still_Ki += 0.0001;
//				break;
//			case 'G':
//				still_Ki += 0.001;
//				break;
//			case 'h':
//				if(still_Ki >= 0.0001)
//					still_Ki -= 0.0001;
//				break;
//			case 'H':
//				if(still_Ki >= 0.001)
//					still_Ki -= 0.001;
//				break;
//
//			case 'j':
//				ideal_count += 3;
//				break;
//			case 'J':
//				ideal_count += 80;
//				break;
//			case 'k':
//				if(ideal_count >= 3)
//					ideal_count -= 3;
//				break;
//			case 'K':
//				if(ideal_count >= 80)
//					ideal_count -= 80;
//				break;
//
//			case 'o':
//				original_angle += 0.01;
//				break;
//			case 'O':
//				original_angle += 0.1;
//				break;
//			case 'p':
//				if(original_angle >= 0.01)
//					original_angle -= 0.01;
//				break;
//			case 'P':
//				if(original_angle >= 0.1)
//					original_angle -= 0.1;
//				break;
//
//			default :
//				return 0;
//
//			}
//		}
//	}
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
