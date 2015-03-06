/*
 * car.h
 *
 *  Created on: 3 Mar, 2015
 *      Author: sfuab
 */

#ifndef INC_CAR_H_
#define INC_CAR_H_


typedef struct Wheel_l{
		DirEncoder* encoder_l;
		int32_t count_l = 0;
		AlternateMotor* motor_l;
		int32_t last_il_encoder_error = 0;
		int32_t il_encoder_error = 0;
		int32_t il_encoder_error_change = 0;
		int32_t il_encoder_errorsum = 0;
		int32_t speed_l = 0;
		int32_t old_speed_l = 0;
	} WHEEL_L;

	typedef struct Wheel_r{
		DirEncoder* encoder_r;
		int32_t count_r = 0;
		AlternateMotor* motor_r;
		int32_t last_ir_encoder_error = 0;
		int32_t ir_encoder_error = 0;
		int32_t ir_encoder_error_change = 0;
		int32_t ir_encoder_errorsum = 0;
		int32_t speed_r = 0;
		int32_t old_speed_r = 0;

	} WHEEL_R;

	typedef struct Common_Para{
		int last_sign = 0;
		int sign = 0;
		float old_ratio = 0.75;
	}  COMMON;

	typedef struct Gyro_Accel{
		Mpu6050* mpu6050;
		std::array<float, 3>accel;
		std::array<float, 3>angle;
		std::array<float, 3>omega;
		double accel_angle;
		double last_gyro_angle;
		double gyro_angle;
		double last_accel_angle;
		double output_angle;
		float trust_gyro = 1;
		float trust_accel = 1 - trust_gyro;
		double last_angle_error = 0;
		double now_angle_error = 0;
		double angle_error_change;
	} GYRO_ACCEL;


	typedef struct Lcd{
		uint16_t result;
		char *buffer = new char[125]{0};
		char *words = new char[125]{0};
		uint16_t pixel1[2100];
		uint16_t pixel_bg_colour[2100];
		uint32_t avg = 0;
		uint32_t all = 0;
		std::array<uint16_t,LinearCcd::kSensorW> pixel;
		float window = 5.0;
		float window_avg = 0;
		int state = 0;

		const char *screen1 = "Interstellar\n\n>Sensor State\n\n Balance Mode\n\n Come Back Mode\n\n Run Forest!!!";
		const char *screen2 ="Interstellar\n\
				\n\
				 Sensor State\n\
				\n\
				>Balance Mode\n\
				\n\
				 Come Back Mode\n\
				\n\
				 Run Forest!!!";
		const char *screen3 = "Interstellar\n\
				\n\
				 Sensor State\n\
				\n\
				 Balance Mode\n\
				\n\
				>Come Back Mode\n\
				\n\
				 Run Forest!!!\n";
		const char *screen4 = "Interstellar\n\
				\n\
				 Sensor State\n\
				\n\
				 Balance Mode\n\
				\n\
				 Come Back Mode\n\
				\n\
				>Run Forest!!!\n";

		const char *screen5 = "Go motor!!!\n";
	} lcd;

	void Balance_function(Wheel_l &, Wheel_r &, float , int32_t &, Gyro_Accel &, Kalman&);
	void Follow_Encoder (Wheel_l &, Wheel_r &, Common_Para &);


#endif /* INC_CAR_H_ */
