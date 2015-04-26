///*
// * app.h
// *
// *  Created on: 9 Mar, 2015
// *      Author: sfuab
// */
//
//#include <car.h>
//
//#ifndef INC_APP_H_
//#define INC_APP_H_
//
//
//
//class App
//{
//public:
//	void Balance_PID();
//	void Encoder_PID();
//
//
//    int Tuner();
//
//private:
//	Car m_car;
//	Timer::TimerInt t, pt;
//	std::array<uint16_t, LinearCcd::kSensorW> Pixel;
//	float trust_accel, trust_old_accel,
//	raw_angle, original_angle, accel_angle, raw_accel_angle,
//	gyro_angle, output_angle, angle_error[3] /* error[0] is now error, error[1] is last error, error[2] is error change */;
//	float ic_Kp, ic_Kd, is_Kp, is_Kd, encoder_l_Kp, encoder_l_Ki, encoder_r_Kp, encoder_r_Ki;
//	float ideal_speed, m_speed, speed_error[3]   /* [0] is now, [1] is last, [2] is change*/;
//	std::array<float, 3>accel, raw_accel, omega;
//	int ideal_count[2], speed[2],
//		/* [0] is now,        /*[0] is left
//		 * [1] is last.        *[1] is right
//		 */
//	count_l, count_r, period[5], sign[2] /* [0] is new while [1] is old. */,
//	encoder_error[4], encoder_error_change[2]  /* [0] is left, [1] is right*/
//	/* [0] is left new,
//	 * [1] is left old,
//	 * [2] is right new,
//	 * [3] is right old.
//	 */;
//	uint16_t ccd_aver;
//
//	void EncoderCommon();
//	void GetAngles();
//	float GetOutputAngle()
//	   {return output_angle = gyro_angle;}                                         // Need to use Kalman Filter later
//	float GetOriginalAngle();
//	void GetIdealCount();
//	void GetEncoderValues();
//	void SetSpeed();
//	void CcdGetValue();
//	void CcdGetAver();
//	void CcdValueFilter();
//
//
//
//
//};
//
//
//
//#endif /* INC_APP_H_ */
