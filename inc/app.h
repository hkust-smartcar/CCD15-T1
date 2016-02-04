/*
 * app.h
 *
 *  Created on: 9 Mar, 2015
 *      Author: sfuab
 */

#include "car.h"

#define TRUST_NOW_SPEED    0.08
#define TRUST_NOW_TURN     0.3
#define RIGHT              1
#define LEFT               0
#define UP_RIGHTANGLE_MAX  15
#define UP_NORMAL_WIDE     45
#define DOWN_NORMAL_WIDE   100
#define BLACK_LINE_MAX     15
#define MIDDLE_LINE_MIN    3
#define MIDDLE_LINE_MAX    7

#ifndef INC_APP_H_
#define INC_APP_H_

class App
{
public:
	int16_t power = 0;
	int16_t encoder_l = 0;
	int16_t encoder_r = 0;
	Car m_car;
	Adverage adv_accel;
	MyVarManager pGrapher;
	MyFlash   m_flash;
	MyVar m_var;
	explicit App(void);
	void Common(const int time);
	void Balance_PID();
	void SetMotors();
	void Speed_PID();
	void Turn_PID();
	void RawAngle();
	void CcdGetValue(int id, Tsl1401cl *ccd);
	void UpCcd();
	void DownCcd();
	void BluetoothSend();
	void pgrapher_setup();
	void gaussian_blur_init(double gaussian_blur_data[],double sigma,const int window);
	void print_ccd(uint16_t color,int ccd_id);
	void gaussian_blur(int id,double gaussian_blur_data[],int window);

private:
	std::array<uint16_t, Tsl1401cl::kSensorW> Data[2];
	Timer::TimerInt t, pt;

	/* Encoders
	 *
	 */
	float 	count_l, count_r,
	ideal_count,
	encoder_error[2]  /*  [0] is left, [1] is right  */;

	/* Angles
	 *
	 */
	std::array<float, 3> omega;
	std::array<float, 3> accel;
	float original_angle, output_angle,accel_angle,gyro_angle,com_filtered,time_interval,gyro_gap,last_omega,current_omega,
	angle_error[2], angle_error_sum, angle_error_change;

	/* PID
	 *
	 */
	float ic_Kp, ic_Kd, ic_Ki,
	is_Kp, is_Kd, is_Ki,
	encoder_l_Kp, encoder_r_Kp,
	turn_Kp, turn_Kd,trust_accel;

	/* Speed
	 *
	 */
	float ideal_speed, m_speed, speed_error_sum, speed_error[3]   /* [0] is now, [1] is last, [2] is change*/;

	/* Motors
	 *
	 */
	float speed_l, speed_r,
	power_l, power_r;

	/* CCD
	 *
	 */
	bool turn_direction;
	int invalid_pixels[2], average_bound[2],     // cannot be initialized here
	ccd_average[2],	white_count[2], black_count, mid_point[2];
	int16_t mid_point_ha,l_edge,r_edge,drop_difference;
	Byte right_angle, black_line, middle_line, cross;
	double gaussian_blur_data[128];

	/* Turn
	 *
	 */
	float turn[2], turn_error[2], turn_error_change,turn_pid_l,turn_pid_r;
	int turn_count;
	int bt_print;
	float raw_angle;
	int border_l[2], border_r[2];
	int l_standard_border, r_standard_border;

	//testing flash
	float *testing;

	void GetOutputAngle();
	void GetIdealCount();

	void FindMidpoint(int id);
	void GetRawAngle();

};



#endif /* INC_APP_H_ */
