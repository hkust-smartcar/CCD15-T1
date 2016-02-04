/*
 * app.cpp
 *
 *  Created on: 9 Mar, 2015
 *      Author: sfuab
 */

//#include <app.h>
#include "app.h"

void App::pgrapher_setup(){
	pGrapher.addWatchedVar(&power,"power");
	pGrapher.addWatchedVar(&encoder_l,"encoder_l");
	pGrapher.addWatchedVar(&encoder_r,"encoder_r");
//	pGrapher.addWatchedVar(&turn[0],"turn[0]");
	//	pGrapher.addWatchedVar(&omega[1],"omega[1]");
//	pGrapher.addSharedVar(&ic_Kd,"ic_Kd");
//	pGrapher.addSharedVar(&ic_Kp,"ic_Kp");
//	pGrapher.addSharedVar(&turn_Kp,"turn_Kp");
	//	pGrapher.addSharedVar(&trust_accel,"trust_accel");
//	pGrapher.addSharedVar(&turn_Kd,"turn_Kd");
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
void App::print_ccd(uint16_t color,int ccd_id) {
	for (uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		m_car.m_lcd->SetRegion(Lcd::Rect(i, (65535 - Data[ccd_id][i])*160/65535,
				1, 1));
		//		m_car.m_lcd->SetRegion(Lcd::Rect(i,160*(1-Data[ccd_id][i]/256),
		//				1, 1));
		m_car.m_lcd->FillColor(color);
	}
	//	m_car.m_lcd->SetRegion(Lcd::Rect(0, 0, 128, 160));
}

void App::gaussian_blur(int id,double gaussian_blur_data[],int window){

	//const int window = 5;
	//int kernel[window] = {1,2,4,2,1};
	//int kernel_sum = 0;
	//for(int i=0;i<window;i++){
	//	kernel_sum += kernel[i];
	//}
	double temp = 0;
	uint16_t temp_data[128];

	for(int i =0;i<128;i++){
		temp = 0;
		if(i<window/2 || i > 127-window/2){
			temp_data[i] = 0;
		}
		else{
			for(int a=i-window/2,z=0;z<window;a++,z++){
				temp += Data[id][a]*gaussian_blur_data[z];
			}
			temp_data[i] = (uint16_t)(temp);
		}
	}

	for(int i=0;i<128;i++){
		Data[id][i] = temp_data[i];
	}
}

void App::GetRawAngle()
{
	int count = 0;
	t = System::Time();
	pt = t;
	while(1){
		if(t != System::Time()){
			t = System::Time();
			if(t%5 == 0){
				m_car.m_mma8451q->Update();
				std::array<float, 3> accel = m_car.m_mma8451q->GetAccel();
				raw_angle += asin(libutil::Clamp<float>(-1,accel[1],1)) * 57.29578;
				count++;
			}
			if((int)(t-pt) >= 2000){
				raw_angle /= count;
				com_filtered = raw_angle;
				return;
			}
		}
	}
}

void App::RawAngle()
{
	GetRawAngle();
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
	static int current_time = 0;
	static float gyro_offset = 0;
	static float mov_adv = 0;
	static int flag = 0;
	int last_time = current_time;
	current_time = System::Time();
	time_interval = (current_time - last_time)/1000.0;
	m_car.m_mpu6050->Update();
	m_car.m_mma8451q->Update();
	omega = m_car.m_mpu6050->GetOmega();
	last_omega = current_omega;
	current_omega = omega[1];
	if(fabsf(current_omega - last_omega) > gyro_gap && flag == 0){
		current_omega = last_omega;
		flag = 1;
	}else{
		flag = 0;
	}
	//	m_car.m_upstand->KalmanFilter();

	accel = m_car.m_mma8451q->GetAccel();
	accel_angle = (float)asin(libutil::Clamp<float>(-1,accel[1],1)) * 57.29578;
	gyro_angle += current_omega*time_interval;
	//	com_filtered += (omega[1]*time_interval + trust_accel*(fabsf(accel_angle-com_filtered)));
	gyro_offset += trust_accel * (accel_angle - com_filtered);

	com_filtered = gyro_angle + gyro_offset;
	output_angle = com_filtered;
	//		output_angle = (float)m_car.m_upstand->GetAngle();
	//	BluetoothSend();
}

void App::GetIdealCount()
{
	if(turn[0] < -0.08)
		original_angle = original_angle + 3.0f + m_speed * 0.04f;
	else if(turn[0] > 0.08)
		original_angle = original_angle + 2.0f + m_speed * 0.04f;

	angle_error[1] = angle_error[0];

	angle_error[0] = tan(libutil::Clamp<float>(-1.0,(raw_angle - output_angle) / 57.29578,1.0));
	angle_error_change = angle_error[0] - angle_error[1];
	angle_error_sum += angle_error[0];
	ideal_count = ic_Kp* angle_error[0] + ic_Kd * angle_error_change + ic_Ki * angle_error_sum;
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
															   + is_Kd * speed_error[2]
																					 + is_Ki * speed_error_sum, 5.5f);
}

void App::SetMotors()
{
	speed_r = ideal_count + encoder_error[0] * encoder_r_Kp;
	speed_l = ideal_count + encoder_error[1] * encoder_l_Kp;

	//	if(right_angle == 11)
	//	{
	//		if(turn_direction == RIGHT)
	//			turn[0] = -0.9f;
	//		else if(turn_direction == LEFT)
	//			turn[0] = 0.9f;
	//	}

	//	power_r = libutil::Clamp<float>(-900.0f, 1.935 * speed_r * (1 + turn[0]), 900.0f);
	//	power_l = libutil::Clamp<float>(-900.0f, 1.745 * speed_l * (1 - turn[0]), 900.0f);



	bool sign = 0;

	if(speed_l >= 0){
		sign = 1;
		power_l = libutil::Clamp<float>(-900.0f, (0.16089069* speed_l*(1 + turn[0]) - 47.6863916091), 900.0f);
	}
	else{
		sign = 0;
		power_l = libutil::Clamp<float>(-900.0f, (0.1925224 * speed_l*(1 - turn[0]) -29.194101), 900.0f);
	}
	m_car.motor_l->SetClockwise(sign);

	if(speed_r >= 0){
		sign = 1;
		power_r = libutil::Clamp<float>(-900.0f, (0.189089 * speed_r*(1 - turn[0]) -53.18521), 900.0f);
	}
	else{
		sign = 0;
		power_r = libutil::Clamp<float>(-900.0f, (0.1597673 * speed_r*(1 + turn[0]) +167.276444), 900.0f);
	}
	m_car.motor_r->SetClockwise(sign);

	//	m_car.motor_l->SetPower(int(abs(speed_l) + 25.3f));
	//	m_car.motor_r->SetPower(int(abs(speed_r) + 27.6f));
	turn_pid_l = abs(power_l);
	turn_pid_r = abs(power_r);
	m_car.motor_l->SetPower(int(libutil::Clamp<float>(170.0f,turn_pid_l,900.0f)));
	m_car.motor_r->SetPower(int(libutil::Clamp<float>(185.0f,turn_pid_r,900.0f)));
}

void App::CcdGetValue(int id, Tsl1401cl *ccd)
{
	ccd->StartSample();
	while(!ccd->SampleProcess()){};
	//	print_ccd(Lcd::kBlack,id);
	Data[id] = ccd->GetData();
	gaussian_blur(id,gaussian_blur_data,9);
	//	print_ccd(Lcd::kGreen,id);

	//	uint16_t ccd_sum = 0;
	//	for(int i = invalid_pixels[id]; i < Tsl1401cl::kSensorW - invalid_pixels[id]; i++)
	//		ccd_sum += Data[id][i];
	//	ccd_average[id] = ccd_sum / (Tsl1401cl::kSensorW - 2 * invalid_pixels[id]);
	//
	//	if(ccd_average[id] > average_bound[id])
	//		ccd_average[id] = ccd_average[id] - 10;
	//	else if(ccd_average[id] < (average_bound[id] - 20))
	//		ccd_average[id] = ccd_average[id] + 10;
	//
	//	white_count[id] = 0;
	//	black_count = 0;
	//	for(int i = invalid_pixels[id]; i < Tsl1401cl::kSensorW - invalid_pixels[id]; i++){
	//		if(Data[id][i] < ccd_average[id])
	//		{
	//			Data[id][i] = 0;
	//			if(i >= 33 && i <= 93)
	//				black_count++;
	//		}
	//		else
	//		{
	//			Data[id][i] = 1;
	//			white_count[id]++;
	//		}
	//	}

	//	if(m_car.border_state == 1)
	//	{
	//		l_standard_border = border_l[1];
	//		r_standard_border = border_r[1];
	//		m_car.border_state = 2;
	//	}
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
	//	if(black_line == 0 &&
	//			middle_line == 0 &&
	//			cross == 00 &&
	//			right_angle == 00)
	//	{
	//		if(abs(border_l[1] - l_standard_border) <= 3 && border_r[1] >= (Tsl1401cl::kSensorW - invalid_pixels[1] - 1))
	//		{
	//			right_angle = 10;
	//			turn_direction = RIGHT;
	//			// Change other states to 0
	//		}
	//
	//		else if(abs(border_r[1] - r_standard_border) <= 3 && border_l[1] <= (invalid_pixels[1] + 1))
	//		{
	//			right_angle = 10;
	//			turn_direction = LEFT;
	//			// Change other states to 0
	//		}
	//	}
	//
	//	//	else if(right_angle == 10)
	//	//	{
	//	//		if(white_count[1] <= UP_RIGHTANGLE_MAX)
	//	//			right_angle = 11;
	//	//	}
	//
	//	else if(right_angle == 11)
	//	{
	//		if(turn_direction == RIGHT)
	//		{
	//			if(abs(border_l[1] - l_standard_border) <= 3)
	//				right_angle = 00;
	//		}
	//
	//		else if(turn_direction == LEFT)
	//		{
	//			if(abs(border_r[1] - r_standard_border) <= 3)
	//				right_angle = 00;
	//		}
	//	}

	// right_angle self-correcting
	//	if(right_angle == 10)
	//	{
	//		if((border_r[1] - border_l[1]) <= UP_NORMAL_WIDE)
	//			right_angle = 00;
	//	}

	FindMidpoint(1);
}



void App::DownCcd()
{
	//	if(white_count[0] <= BLACK_LINE_MAX && black_line == 0 && right_angle == 0)
	//	{
	//		black_line = 1;
	//		middle_line = 0;
	//		//						if(ideal_speed == LOW_IDEAL_SPEED)
	//		//							ideal_speed = HIGH_IDEAL_SPEED;
	//		//						else
	//		//							ideal_speed = LOW_IDEAL_SPEED;
	//	}
	//
	//	else if(white_count[0] >= (DOWN_NORMAL_WIDE / 3) && black_line == 1)
	//		black_line = 0;
	//
	//	 //middle line detection
	//	if(black_count >= MIDDLE_LINE_MIN && black_count <= MIDDLE_LINE_MAX && middle_line == 0 && black_line == 0 && right_angle == 0)   // added right_angle
	//	{
	//		int temp_border_l= 0, temp_border_r = 0;
	//
	//		for(int i = 33; i <= 93; i++)
	//		{
	//			temp_border_l = i;
	//			if(Data[0][i] == 0)
	//				break;
	//		}
	//
	//		for(int i = 93; i >= 33; i--)
	//		{
	//			temp_border_r = i;
	//			if(Data[0][i] == 0)
	//				break;
	//		}
	//		int temp_mid_point = (temp_border_r + temp_border_l) / 2;
	//
	//		if(border_l <= border_r && (border_r - border_l) <= 8
	//				&& Data[0][libutil::Clamp<int>(invalid_pixels[0], temp_mid_point + 10, (Tsl1401cl::kSensorW - invalid_pixels[0] - 1))] == 1
	//				&& Data[0][libutil::Clamp<int>(invalid_pixels[0], temp_mid_point - 10, (Tsl1401cl::kSensorW - invalid_pixels[0] - 1))] == 1)
	//		{
	//			middle_line = 1;
	//			// Change other states to 0
	//			cross = 00;
	//		}
	//	}
	//
	//	if(middle_line == 1)
	//	{
	//		for(int i = mid_point[0] - 30; i <= mid_point[0] + 30; i++)
	//		{
	//			border_l[0] = i;
	//			if(Data[0][i] == 0)
	//				break;
	//		}
	//
	//		for(int i = mid_point[0] + 30; i >= mid_point[0] - 30; i--)
	//		{
	//			border_r[0] = i;
	//			if(Data[0][i] == 0)
	//				break;
	//		}
	//
	//		if(border_l[0] > border_r[0] || (border_r[0] - border_l[0])> MIDDLE_LINE_MAX)
	//		{
	//			middle_line = 0;
	//			border_l[0] = mid_point[0];
	//			border_r[0] = mid_point[0];
	//		}
	//	}
	//
	//	if(right_angle == 10)
	//	{
	//		if(turn_direction == RIGHT)
	//		{
	//			if(border_r[0] >= (Tsl1401cl::kSensorW - invalid_pixels[0] - 3))
	//				right_angle = 11;
	//		}
	//		else if(turn_direction == LEFT)
	//		{
	//			if(border_l[0] >= (invalid_pixels[0] + 2))
	//				right_angle = 11;
	//		}
	//	}

	FindMidpoint(0);

	if(white_count[0] >= (Tsl1401cl::kSensorW - 2 * invalid_pixels[0] - 2) && cross == 10)
		cross = 11;

	// cross self-correcting
	//	else if(cross == 11 && white_count[0] < (DOWN_NORMAL_WIDE + 5))
	//		cross = 00;
}



void App::FindMidpoint(int id)
{
	//	m_car.m_lcd->SetRegion(Lcd::Rect(border_l[id],146,2, 14));
	//	m_car.m_lcd->FillColor(Lcd::kBlack);
	//	m_car.m_lcd->SetRegion(Lcd::Rect(border_r[id],146,2, 14));
	//	m_car.m_lcd->FillColor(Lcd::kBlack);
	static int drop_sum = 0;
	static int continuous_flag = 0;
	//	static int current_edge_l = 0;
	//	static int current_edge_r = 127;
	int last_edge_l = border_l[id];
	int last_edge_r = border_r[id];

	if(last_edge_r - last_edge_l > 0){
		for(int i = border_r[id];i > invalid_pixels[id];i--){
			border_l[id] = 0;
			if(Data[id][i]-Data[id][i-1]>-200){
				continuous_flag = 1;
			}else{
				continuous_flag = 0;
			}
			if(continuous_flag == 1){
				drop_sum += Data[id][i]-Data[id][i-1];
			}else{
				drop_sum = 0;
			}
			if(drop_sum > drop_difference){
				border_l[id] = i;
				break;
			}
		}
		drop_sum = 0;
		continuous_flag = 0;
		for(int i = border_l[id];i < 127 - invalid_pixels[id];i++){
			border_r[id] = 127;
			if(Data[id][i]-Data[id][i+1]>-200){
				continuous_flag = 1;
			}else{
				continuous_flag = 0;
			}
			if(continuous_flag == 1){
				drop_sum += Data[id][i]-Data[id][i+1];
			}else{
				drop_sum = 0;
			}
			if(drop_sum > drop_difference){
				border_r[id] = i;
				break;
			}
		}
	}else{
		border_l[id] = 0;
		border_r[id] = 127;
	}
	//	for(int i = mid_point[id]; i >= invalid_pixels[id]; i--)
	//	{
	//		border_l[id] = i;
	//		if(Data[id][i] == 0)
	//			break;
	//	}
	//
	//	for(int i = mid_point[id]; i < Tsl1401cl::kSensorW - invalid_pixels[id]; i++)
	//	{
	//		border_r[id] = i;
	//		if(Data[id][i] == 0)
	//			break;
	//	}
	//

	//	m_car.m_lcd->SetRegion(Lcd::Rect(border_l[id],146,2, 14));
	//	m_car.m_lcd->FillColor(Lcd::kBlue);
	//	m_car.m_lcd->SetRegion(Lcd::Rect(border_r[id],146,2, 14));
	//	m_car.m_lcd->FillColor(Lcd::kBlue);
	//
	//
	//	m_car.m_lcd->SetRegion(Lcd::Rect(mid_point[id],0,2, 160));
	//	m_car.m_lcd->FillColor(Lcd::kBlack);
	mid_point[id] = (border_l[id] + border_r[id])/2;
	//	m_car.m_lcd->SetRegion(Lcd::Rect(mid_point[id],0,2, 160));
	//	m_car.m_lcd->FillColor(Lcd::kRed);

	mid_point_ha = (int16_t)mid_point[id];
	//	l_edge = 1;
	//	r_edge = border_r[id];
}


void App::Turn_PID()
{
	turn_error[1] = turn_error[0];
	//	if(black_line == 1)
	//		turn_error[0] = float(63 - mid_point[1]);
	//	else
	turn_error[0] = float(mid_point[0]- 63);
	turn_error_change = turn_error[0] - turn_error[1];
	turn[0] = turn_Kp*turn_error[0] + turn_Kd*turn_error_change;
	turn[0] = libutil::Clamp<float>(-2.0f,turn[0],2.0f);
	//	double temp_ = 0;
	//	m_car.turnKF->Filtering(&temp_, turn_error[0],0);
	//	turn_error[0] = temp_;
	//
	//	turn_error_change = turn_error[0] - turn_error[1];
	//
	//	float hehe = turn_Kp;
	//
	//	if(right_angle == 10 || cross == 11)
	//		hehe = hehe / 50;
	//
	//	turn_count++;
	//	if(turn_count >= 2)
	//	{
	//		turn_count = 0;
	//		turn[1] = TRUST_NOW_TURN * turn[0] + (1 - TRUST_NOW_TURN) * turn[1];
	//		turn[0] = libutil::Clamp<float>(-0.4f, hehe * turn_error[0] + turn_error_change * turn_Kd, 0.4f);
	//		turn[0] = TRUST_NOW_TURN * turn[0] + (1 - TRUST_NOW_TURN) * turn[1];
	//
	//		if(cross != 11)
	//		{
	//			if(turn[0] > 0.07)
	//				turn[0] += 0.12f;
	//			else if(turn[0] < 0.07)
	//				turn[0] -= 0.12f;
	//		}
	//	}

	//	if(right_angle == 11)
	//	{
	//		if(turn_direction == RIGHT)
	//			turn[0] = -0.9f;
	//		else if(turn_direction == LEFT)
	//			turn[0] = 0.9f;
	//	}
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
	pGrapher.sendWatchData();
}





App::App(void):
																																																					m_car(),
																																																					t(0), pt(0), count_l(0), count_r(0),
																																																					ideal_count(0),encoder_error{0, 0},
																																																					original_angle(0), output_angle(0),accel_angle(0),gyro_angle(0),com_filtered(0),
																																																					angle_error{0, 0}, angle_error_sum(0), angle_error_change(0),gyro_gap(20),current_omega(0),last_omega(0),
																																																					ic_Kp(0), ic_Kd(0), ic_Ki(0),
																																																					is_Kp(0), is_Kd(0), is_Ki(0),
																																																					encoder_l_Kp(0), encoder_r_Kp(0),
																																																					turn_Kp(0), turn_Kd(0),trust_accel(0.008),
																																																					ideal_speed(0), m_speed(0), speed_error_sum(0),
																																																					speed_error{0, 0, 0},speed_l(0), speed_r(0),
																																																					power_l(0), power_r(0), turn_direction(LEFT),
																																																					invalid_pixels{7, 20}, average_bound{220, 200},
																																																					ccd_average{0, 0}, white_count{0, 0}, black_count(0), mid_point{64, 64},drop_difference(8000),
																																																					right_angle(0), black_line(0), middle_line(0), cross(0),
																																																					turn{0, 0}, turn_error{0, 0},
																																																					turn_error_change(0), turn_count(0),
																																																					bt_print(0), raw_angle(0),
																																																					border_l{0, 0}, border_r{127, 127},
																																																					l_standard_border(0), r_standard_border(0),
																																																					adv_accel(10,3),pGrapher(),m_flash(),m_var(),mid_point_ha(0),l_edge(0),r_edge(0)
																																																					{
																																																						gaussian_blur_init(gaussian_blur_data,2.5,9 );
																																																						//						m_flash.eraseAll();
																																																						//												pGrapher.SetOnChangedListener(&m_flash.writeConfig);

																																																					}




























