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
//	float original_angle, accel_angle, gyro_angle, output_angle, angle_error[3] /* error[0] is now error, error[1] is last error, error[2] is error change */;
//	float total_count_r, total_count_l, ic_Kp, ic_Kd, still_Ki;
//	std::array<float, 3>accel;
//	std::array<float, 3>omega;
//	int dead_value_l = 200;
//	int dead_value_r = 200;
//	int ideal_count[2], speed[2],
//		/* [0] is now,        /*[0] is left
//		 * [1] is last.        *[1] is right
//		 */
//	count_l, count_r, period[5], sign[2] /* [0] is new while [1] is old. */,
//	encoder_error[4], encoder_error_change[4]
//	/* [0] is left new,
//	 * [1] is left old,
//	 * [2] is right new,
//	 * [3] is right old.
//	 */;
//	uint16_t ccd_aver;
//
//	void GetAngles();
//	float GetOutputAngle()
//	   {return output_angle = gyro_angle;}                                         // Need to use Kalman Filter later
//	float GetOriginalAngle();
//	int DeadZoneTesting(int&, int&);
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
