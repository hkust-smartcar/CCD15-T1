/*
 * MyVar.h
 *
 * Author: Lincoln
 * Copyright (c) 2015-2016 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include <array>
#include <cassert>
#include <stdint.h>
#include <libsc/system.h>
#include <libsc/k60/config/2015_magnetic.h>

#define MAX(a, b) ((a > b)? a : b)
#define inRange(n, v, x) ((v < n)? n : ((v > x)? x : v))
#define ABS(v) ((v > 0)? v : -v)


class MyVar
{

public:

	class ConfigTable
	{

	public:

		struct testing
		{
			static float test;
		};
		//		struct BuzzerConfig
		//		{
		////			static uint8_t WarningPitch;
		////			static uint8_t WarningTimes;
		//		};
		//
		//		struct MagSenConfig
		//		{
		//			static float Kq;
		//			static float Kr;
		//		};
		//
		//		struct ServoConfig
		//		{
		//			static float Reference;
		//
		//
		//			static float NormalKp;
		////			static float NormalKi;
		//			static float NormalKd;
		//
		////			static float NormalWeightSD;
		////			static float NormalWeightFD;
		////			static float NormalWeightHD;
		//
		//
		//			static float TurningKpA;
		//			static float TurningKpB;
		////			static float TurningKi;
		//			static float TurningKd;
		//
		////			static float TurningWeightSD;
		////			static float TurningWeightFD;
		////			static float TurningWeightHD;
		//
		//			static float Turning90DegreeThresholdSdAvg;
		//			static float Turning90DegreeThresholdSd;
		//			static float SdNoSignalThreshold;
		//			static float FdNoSignalThreshold;
		//			static float Turning90DegreeThresholdFd;
		//			static float Turning90DegreeThresholdFdAvg;
		//
		//
		//			static uint32_t UpdateFreq;
		//		};
		//
		//		struct MotorConfig
		//		{
		//			static float Reference;
		//			static float Kp;
		////			static float Ki;
		//			static float Kd;
		//
		//			static float TurningSpeedRatio;
		//			static float Turning90DegreeSpeedRatio;
		////			static float MaxSpeedRatio;
		////
		////			static uint32_t TimeForReachingMaxSpeed;
		////			static int16_t EmergencyStopThreshold;
		//
		//			static uint32_t UpdateFreq;
		//		};
		//
		//		struct VarMngConfig
		//		{
		//
		//		public:
		//
		//			static uint16_t UpdateFreq;
		//
		//		};
		//
		//		struct BatteryMeterConfig
		//		{
		//
		//		public:
		//
		//			static float MinVoltage;
		//			static float MaxVoltage;
		//			static float Ratio;
		//			static uint16_t UpdateFreq;
		//
		//		};
		//
		//		struct EncoderConfig
		//		{
		//
		//		public:
		//
		//			static float UpdateFreq;
		//
		//		};
		//
		//		struct LcdConfig
		//		{
		//
		//		public:
		//
		//			static uint8_t Fps;
		//			static uint16_t BgColor;
		//			static uint16_t TxtColor;
		//
		////			static uint16_t HighPowerColor;
		////			static uint16_t NormalPowerColor;
		////			static uint16_t LowPowerColor;
		//
		//			static uint16_t UpdateFreq;
		//
		//		};

	};

	//	MyVar(App *app);
	MyVar();
	//	static App &my_app(void);
	//	void init(void);

	//private:
	//
	//	App *m_app;

};
