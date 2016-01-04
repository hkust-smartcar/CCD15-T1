/*
 * app.cpp
 *
 *  Created on: 9 Mar, 2015
 *      Author: sfuab
 */

//#include <app.h>
#include "app.h"

void App::moving_adverage(uint32_t data[],int window){
	int temp = 0;
	int temp_data[128];
	for(int i =0;i<128;i++){
		temp = 0;
		if(i==0 || i == 127){

		}
		else{
			for(int a=i-1;(a-i+1)<window;a++){
				temp += data[a];
			}
			temp_data[i] = (uint32_t)(temp/(float)window);
		}
	}
	for(int i=0;i<128;i++){
		data[i] = temp_data[i];
	}
}
void App::gaussian_blur_init(double gaussian_blur_data[],double sigma,const int window){
	//double kernel[window];
	int origin = window/2;
	double sum = 0.0; // For accumulating the kernel values
	for (int x = 0; x < window; ++x){
		gaussian_blur_data[x] = exp( -0.5 * (pow((x-origin)/sigma, 2.0)) )
                        		 / (sigma*pow(2*3.14159,0.5));
		// Accumulate the kernel values
		sum += gaussian_blur_data[x];
	}
	// Normalize the kernel
	for (int x = 0; x < window; ++x)
		gaussian_blur_data[x] /= sum;

}

void App::gaussian_blur(uint32_t data[],double gaussian_blur_data[],int window){

	//const int window = 5;
	//int kernel[window] = {1,2,4,2,1};
	//int kernel_sum = 0;
	//for(int i=0;i<window;i++){
	//	kernel_sum += kernel[i];
	//}
	double temp = 0;
	uint32_t temp_data[128];
	for(int i =0;i<128;i++){
		temp = 0;
		if(i<window/2 || i > 127-window/2){

		}
		else{
			for(int a=i-window/2,z=0;z<window;a++,z++){
				temp += data[a]*gaussian_blur_data[z];
			}
			temp_data[i] = (uint32_t)(temp);
		}
	}

	for(int i=0;i<128;i++){
		data[i] = temp_data[i];
	}
}
void App::pgrapher_setup(){
	//	pGrapher.addWatchedVar(&output_angle,"output_angle");
	//	pGrapher.addWatchedVar(&original_angle,"original_angle");
	//	//		pGrapher.addWatchedVar(&accel_angle,"accel_angle");
	//	//		pGrapher.addWatchedVar(&gyro_angle,"gyro_angleaw");
	//	//	pGrapher.addWatchedVar(&ideal_speed,"ideal_speed");
	//
	//	pGrapher.addSharedVar(&ic_Kd,"ic_Kd");
	//	pGrapher.addSharedVar(&ic_Kp,"ic_Kp");
	//	pGrapher.addSharedVar(&ic_Ki,"ic_Ki");
	//	//				pGrapher.addSharedVar(&trust_accel,"trust_accel");
}
float App::GetRawAngle()
{
	t = System::Time();
	pt = t;
	while(1){
		t = System::Time();
		m_car.m_mma8451q->Update();
		std::array<float, 3> accel = m_car.m_mma8451q->GetAccel();
		raw_angle = accel[1] * 57.29578;
		if((int)(t-pt) >= 2000)
			return raw_angle;
	}
}

void App::RawAngle()
{
	output_angle = GetRawAngle();
}

void App::Common(const int time)
{
	m_car.encoder_r->Update();
	m_car.encoder_l->Update();

	count_r = (float)(-m_car.encoder_r->GetCount());
	count_l = (float)( m_car.encoder_l->GetCount());

	encoder_error[0] = ideal_count - count_r;
	encoder_error[1] = ideal_count - count_l;

	m_speed = (count_l + count_r)/ (2 * time);

	double _temp = 0;
	m_car.speedKF->Filtering(&_temp, m_speed, 0);
	m_speed = _temp;
}

void App::GetOutputAngle()
{
	m_car.m_mpu6050->Update();
	m_car.m_mma8451q->Update();
	omega = m_car.m_mpu6050->GetOmega();
	m_car.m_upstand->KalmanFilter();

	output_angle = (float)m_car.m_upstand->GetAngle();
}

void App::GetIdealCount()
{
	if(turn[0] < -0.08)
		original_angle = original_angle + 3.0f + m_speed * 0.04f;
	else if(turn[0] > 0.08)
		original_angle = original_angle + 2.0f + m_speed * 0.04f;

	angle_error[1] = angle_error[0];
	angle_error[0] = tan((original_angle - output_angle) / 57.29578);
	angle_error_change = angle_error[0] - angle_error[1];
	angle_error_sum += angle_error[0];
	ideal_count = m_car.ic_Kp* angle_error[0]
										   + m_car.ic_Kd * angle_error_change
										   + ic_Ki * angle_error_sum;
}

void App::Balance_PID()
{
	GetOutputAngle();
	GetIdealCount();
}

void App::Speed_PID()
{
	// last_speed_error = TRUST_NOW_SPEED * speed_error + (1 - TRUST_NOW_SPEED) * last_speed_error;
	speed_error[1] = TRUST_NOW_SPEED * speed_error[0] + (1 - TRUST_NOW_SPEED) * speed_error[1];
	speed_error[0] = TRUST_NOW_SPEED * (ideal_speed - m_speed) + (1 - TRUST_NOW_SPEED) * speed_error[1];               // speed_error = ideal_speed - speed;
	speed_error[2] = speed_error[0] - speed_error[1];     // speed_error_change = (speed_error - last_speed_error)
	speed_error_sum += speed_error[0];

	float anti_friction_angle = 0.0f;

	if(ideal_speed != 0)
		anti_friction_angle = ideal_speed * 0.0017f;
	else
		anti_friction_angle = 0;

	original_angle = raw_angle - anti_friction_angle + m_car.car_raw_angle
			- libutil::Clamp<float>(-9.0f, is_Kp * speed_error[0]
															   + m_car.is_Kd * speed_error[2]
																						   + is_Ki * speed_error_sum, 5.5f);
}

void App::SetMotors()
{
	speed_r = ideal_count + encoder_error[0] * encoder_r_Kp;
	speed_l = ideal_count + encoder_error[1] * encoder_l_Kp;

	if(right_angle == 11)
	{
		if(turn_direction == RIGHT)
			turn[0] = -0.9f;
		else if(turn_direction == LEFT)
			turn[0] = 0.9f;
	}

	power_r = libutil::Clamp<float>(-900.0f, 1.935 * speed_r * (1 + turn[0]), 900.0f);
	power_l = libutil::Clamp<float>(-900.0f, 1.745 * speed_l * (1 - turn[0]), 900.0f);

	bool sign = 0;

	if(power_l >= 0) sign = 0;
	else             sign = 1;
	m_car.motor_l->SetClockwise(sign);

	if(power_r >= 0) sign = 1;
	else             sign = 0;
	m_car.motor_r->SetClockwise(sign);

	m_car.motor_l->SetPower(int(abs(speed_l) + 25.3f));
	m_car.motor_r->SetPower(int(abs(speed_r) + 27.6f));
}

void App::CcdGetValue(int id, Tsl1401cl *ccd)
{
	while(!ccd->SampleProcess()){};
	Data[id] = ccd->GetData();
	ccd->StartSample();

	uint32_t ccd_sum = 0;
	for(int i = invalid_pixels[id]; i < Tsl1401cl::kSensorW - invalid_pixels[id]; i++)
		ccd_sum += Data[id][i];
	ccd_average[id] = ccd_sum / (Tsl1401cl::kSensorW - 2 * invalid_pixels[id]);

	if(ccd_average[id] > average_bound[id])
		ccd_average[id] = ccd_average[id] - 10;
	else if(ccd_average[id] < (average_bound[id] - 20))
		ccd_average[id] = ccd_average[id] + 10;

	white_count[id] = 0;
	black_count = 0;
	for(int i = invalid_pixels[id]; i < Tsl1401cl::kSensorW - invalid_pixels[id]; i++){
		if(Data[id][i] < ccd_average[id])
		{
			Data[id][i] = 0;
			if(i >= 33 && i <= 93)
				black_count++;
		}
		else
		{
			Data[id][i] = 1;
			white_count[id]++;
		}
	}

	if(m_car.border_state == 1)
	{
		l_standard_border = border_l[1];
		r_standard_border = border_r[1];
		m_car.border_state = 2;
	}
}

void App::UpCcd()
{
	if(black_line == 0 && middle_line == 0 && cross == 00 && white_count[1] >= (Tsl1401cl::kSensorW - 2 * invalid_pixels[1] - 1))
	{
		cross = 10;
		right_angle = 00;
	}

	// cross self_correcting
	if(cross == 10 && white_count[1] <= (UP_NORMAL_WIDE + 5))        // normally wide is 55
		cross = 00;

	// Detect the right angle
	if(black_line == 0 &&
			middle_line == 0 &&
			cross == 00 &&
			right_angle == 00)
	{
		if(abs(border_l[1] - l_standard_border) <= 3 && border_r[1] >= (Tsl1401cl::kSensorW - invalid_pixels[1] - 1))
		{
			right_angle = 10;
			turn_direction = RIGHT;
			// Change other states to 0
		}

		else if(abs(border_r[1] - r_standard_border) <= 3 && border_l[1] <= (invalid_pixels[1] + 1))
		{
			right_angle = 10;
			turn_direction = LEFT;
			// Change other states to 0
		}
	}

	//	else if(right_angle == 10)
	//	{
	//		if(white_count[1] <= UP_RIGHTANGLE_MAX)
	//			right_angle = 11;
	//	}

	else if(right_angle == 11)
	{
		if(turn_direction == RIGHT)
		{
			if(abs(border_l[1] - l_standard_border) <= 3)
				right_angle = 00;
		}

		else if(turn_direction == LEFT)
		{
			if(abs(border_r[1] - r_standard_border) <= 3)
				right_angle = 00;
		}
	}

	// right_angle self-correcting
	if(right_angle == 10)
	{
		if((border_r[1] - border_l[1]) <= UP_NORMAL_WIDE)
			right_angle = 00;
	}

	FindMidpoint(1);
}



void App::DownCcd()
{
	if(white_count[0] <= BLACK_LINE_MAX && black_line == 0 && right_angle == 0)
	{
		black_line = 1;
		middle_line = 0;
		//						if(ideal_speed == LOW_IDEAL_SPEED)
		//							ideal_speed = HIGH_IDEAL_SPEED;
		//						else
		//							ideal_speed = LOW_IDEAL_SPEED;
	}

	else if(white_count[0] >= (DOWN_NORMAL_WIDE / 3) && black_line == 1)
		black_line = 0;

	// middle line detection
	if(black_count >= MIDDLE_LINE_MIN && black_count <= MIDDLE_LINE_MAX && middle_line == 0 && black_line == 0 && right_angle == 0)   // added right_angle
	{
		int temp_border_l= 0, temp_border_r = 0;

		for(int i = 33; i <= 93; i++)
		{
			temp_border_l = i;
			if(Data[0][i] == 0)
				break;
		}

		for(int i = 93; i >= 33; i--)
		{
			temp_border_r = i;
			if(Data[0][i] == 0)
				break;
		}
		int temp_mid_point = (temp_border_r + temp_border_l) / 2;

		if(border_l <= border_r && (border_r - border_l) <= 8
				&& Data[0][libutil::Clamp<int>(invalid_pixels[0], temp_mid_point + 10, (Tsl1401cl::kSensorW - invalid_pixels[0] - 1))] == 1
				&& Data[0][libutil::Clamp<int>(invalid_pixels[0], temp_mid_point - 10, (Tsl1401cl::kSensorW - invalid_pixels[0] - 1))] == 1)
		{
			middle_line = 1;
			// Change other states to 0
			cross = 00;
		}
	}

	if(middle_line == 1)
	{
		for(int i = mid_point[0] - 30; i <= mid_point[0] + 30; i++)
		{
			border_l[0] = i;
			if(Data[0][i] == 0)
				break;
		}

		for(int i = mid_point[0] + 30; i >= mid_point[0] - 30; i--)
		{
			border_r[0] = i;
			if(Data[0][i] == 0)
				break;
		}

		if(border_l[0] > border_r[0] || (border_r[0] - border_l[0])> MIDDLE_LINE_MAX)
		{
			middle_line = 0;
			border_l[0] = mid_point[0];
			border_r[0] = mid_point[0];
		}
	}

	if(right_angle == 10)
	{
		if(turn_direction == RIGHT)
		{
			if(border_r[0] >= (Tsl1401cl::kSensorW - invalid_pixels[0] - 3))
				right_angle = 11;
		}
		else if(turn_direction == LEFT)
		{
			if(border_l[0] >= (invalid_pixels[0] + 2))
				right_angle = 11;
		}
	}

	FindMidpoint(0);

	if(white_count[0] >= (Tsl1401cl::kSensorW - 2 * invalid_pixels[0] - 2) && cross == 10)
		cross = 11;

	// cross self-correcting
	//	else if(cross == 11 && white_count[0] < (DOWN_NORMAL_WIDE + 5))
	//		cross = 00;
}



void App::FindMidpoint(int id)
{
	for(int i = mid_point[id]; i >= invalid_pixels[id]; i--)
	{
		border_l[id] = i;
		if(Data[id][i] == 0)
			break;
	}

	for(int i = mid_point[id]; i < Tsl1401cl::kSensorW - invalid_pixels[id]; i++)
	{
		border_r[id] = i;
		if(Data[id][i] == 0)
			break;
	}

	mid_point[id] = (border_l[id] + border_r[id]) / 2;
}


void App::Turn_PID()
{
	turn_error[1] = turn_error[0];
	if(black_line == 1)
		turn_error[0] = 63 - mid_point[1];
	else
		turn_error[0] = 63 - mid_point[0];

	double temp_ = 0;
	m_car.turnKF->Filtering(&temp_, turn_error[0],0);
	turn_error[0] = temp_;

	turn_error_change = turn_error[0] - turn_error[1];

	float hehe = turn_Kp;

	if(right_angle == 10 || cross == 11)
		hehe = hehe / 50;

	turn_count++;
	if(turn_count >= 2)
	{
		turn_count = 0;
		turn[1] = TRUST_NOW_TURN * turn[0] + (1 - TRUST_NOW_TURN) * turn[1];
		turn[0] = libutil::Clamp<float>(-0.4f, hehe * turn_error[0] + turn_error_change * turn_Kd, 0.4f);
		turn[0] = TRUST_NOW_TURN * turn[0] + (1 - TRUST_NOW_TURN) * turn[1];

		if(cross != 11)
		{
			if(turn[0] > 0.07)
				turn[0] += 0.12f;
			else if(turn[0] < 0.07)
				turn[0] -= 0.12f;
		}
	}

	if(right_angle == 11)
	{
		if(turn_direction == RIGHT)
			turn[0] = -0.9f;
		else if(turn_direction == LEFT)
			turn[0] = 0.9f;
	}
}


void App::BluetoothSend()                     //howard
{
	//	if(angle_error_sum >= 3)
	//		angle_error_sum = 3;
	//	else if(angle_error_sum <= -3)
	//		angle_error_sum = -3;
	//
	//	if(bt_print != m_car.car_bt_print)
	//		bt_print = m_car.car_bt_print;
	//
	//	if(bt_print == 0)
	////					printf("%.2f, %.2f, %d, %d, %d, %d\n", ideal_speed->GetReal(), m_speed, cross, right_angle, middle_line, black_line);
	////					printf(" %f, %f, %f, %f\n", accel_angle, gyro_angle, output_angle, merged_output_angle);
	//		printf("%f, %f, %f, %f\n", m_car.ideal_speed->GetReal(), m_speed, original_angle, output_angle);
	//
	//
	//	else if(bt_print == 1)
	//		printf("%d, %d, %d, %d, %d, %d\n", mid_point[1], border_l[1], border_r[1], l_standard_border, r_standard_border, right_angle);
	//
	//	else if(bt_print == 2)
	//	{
	//		Byte buf[128 + 2];
	//		buf[0] = 'L';
	//		for(int i=0; i<128; i++){
	//			buf[i + 1] = Data[0][i];
	//		}
	//		buf[sizeof(buf) - 1] = '\n';
	//		m_car.m_bt->SendBuffer(buf, sizeof(buf));
	//	}
	//
	//	else if(bt_print == 3)
	//	{
	//		Byte buf[128 + 2];
	//		buf[0] = 'L';
	//		for(int i=0; i<128; i++){
	//			buf[i + 1] = Data[1][i];
	//		}
	//		buf[sizeof(buf) - 1] = '\n';
	//		m_car.m_bt->SendBuffer(buf, sizeof(buf));
	//	}
	//	pGrapher.sendWatchData();
}





App::App():
						m_car(),
						t(0), pt(0), count_l(0), count_r(0),
						ideal_count(0),encoder_error{0, 0},
						original_angle(0), output_angle(0),
						angle_error{0, 0}, angle_error_sum(0), angle_error_change(0),
						ic_Kp(0), ic_Kd(0), ic_Ki(0),
						is_Kp(0), is_Kd(0), is_Ki(0),
						encoder_l_Kp(0), encoder_r_Kp(0),
						turn_Kp(0), turn_Kd(0),
						ideal_speed(0), m_speed(0), speed_error_sum(0),
						speed_error{0, 0, 0},speed_l(0), speed_r(0),
						power_l(0), power_r(0), turn_direction(LEFT),
						invalid_pixels{7, 20}, average_bound{220, 200},
						ccd_average{0, 0}, white_count{0, 0}, black_count(0), mid_point{0, 0},
						right_angle(0), black_line(0), middle_line(0), cross(0),
						turn{0, 0}, turn_error{0, 0},
						turn_error_change(0), turn_count(0),
						bt_print(0), raw_angle(0),
						border_l{0, 0}, border_r{0, 0},
						l_standard_border(0), r_standard_border(0)
						{
							gaussian_blur_init(gaussian_blur_data,2.5,9 );
						}




























