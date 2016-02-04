/*
 * MyVar.cpp
 *
 * Author: Lincoln
 * Copyright (c) 2015-2016 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <libsc/system.h>
#include "car.h"

//MyVar *m_resInstance = nullptr;

//MyVar::MyVar(App *app)
//{
//	System::Init();
//	if (!m_resInstance && app)
//	{
//		m_resInstance = this;
//		m_app = app;
//	}
//}
MyVar::MyVar(void)
{

}
float MyVar::ConfigTable::testing::test = 0.0;
//App &MyVar::my_app(void)
//{
//	return *m_resInstance->m_app;
//}


////#ifdef IS_FRONT
////
////#if IS_FRONT == 1
//
//	// Buzzer
////	uint8_t MyVar::ConfigTable::BuzzerConfig::WarningPitch = 70;
////	uint8_t MyVar::ConfigTable::BuzzerConfig::WarningTimes = 3;
//
//	// MagSen
//	float MyVar::ConfigTable::MagSenConfig::Kq = 0.00015f;
//	float MyVar::ConfigTable::MagSenConfig::Kr = 1.70f;
//
//	// Servo
////	float MyVar::ConfigTable::ServoConfig::Reference = 0.0f;
//
//
//	float MyVar::ConfigTable::ServoConfig::NormalKp = 174.0f;
////	float MyVar::ConfigTable::ServoConfig::NormalKi = 0.0f;
//	float MyVar::ConfigTable::ServoConfig::NormalKd = 9.0f;
//
////	float MyVar::ConfigTable::ServoConfig::NormalWeightSD = 0.3589743f;
////	float MyVar::ConfigTable::ServoConfig::NormalWeightFD = 0.1794872f;
////	float MyVar::ConfigTable::ServoConfig::NormalWeightHD = 0.4615385f;
//
//
//	float MyVar::ConfigTable::ServoConfig::TurningKpA =  328.0f; //250.0f;
//	float MyVar::ConfigTable::ServoConfig::TurningKpB = 308.0f; //120.0f;
////	float MyVar::ConfigTable::ServoConfig::TurningKi = 0.0f;
//	float MyVar::ConfigTable::ServoConfig::TurningKd = 30.0f;
//
////	float MyVar::ConfigTable::ServoConfig::TurningWeightSD = 0.1f;
////	float MyVar::ConfigTable::ServoConfig::TurningWeightFD = 0.325f;
////	float MyVar::ConfigTable::ServoConfig::TurningWeightHD = 0.525f;
//
//	float MyVar::ConfigTable::ServoConfig::Turning90DegreeThresholdSdAvg = 0.7f;
//	float MyVar::ConfigTable::ServoConfig::Turning90DegreeThresholdSd = 0.2f;
//	float MyVar::ConfigTable::ServoConfig::SdNoSignalThreshold = 0.675f;
//	float MyVar::ConfigTable::ServoConfig::FdNoSignalThreshold = 0.3f;
//	float MyVar::ConfigTable::ServoConfig::Turning90DegreeThresholdFd = 0.4f;
//	float MyVar::ConfigTable::ServoConfig::Turning90DegreeThresholdFdAvg = 1.0f;
//
//
//	uint32_t MyVar::ConfigTable::ServoConfig::UpdateFreq = 2;
//
//	// Motor
//	float MyVar::ConfigTable::MotorConfig::Reference = 1300.0f;
//	float MyVar::ConfigTable::MotorConfig::Kp = 0.025f;
////	float MyVar::ConfigTable::MotorConfig::Ki = 0.0f;
//	float MyVar::ConfigTable::MotorConfig::Kd = 0.018f;
//
//	float MyVar::ConfigTable::MotorConfig::TurningSpeedRatio = 0.8f;
////	float MyVar::ConfigTable::MotorConfig::Turning90DegreeSpeedRatio = 0.65f;
////	float MyVar::ConfigTable::MotorConfig::MaxSpeedRatio = 1.3f;
////
////	uint32_t MyVar::ConfigTable::MotorConfig::TimeForReachingMaxSpeed = 3000;
////	int16_t MyVar::ConfigTable::MotorConfig::EmergencyStopThreshold = 0;
//
//	uint32_t MyVar::ConfigTable::MotorConfig::UpdateFreq = 20;
//
//	// VarMng
//	uint16_t MyVar::ConfigTable::VarMngConfig::UpdateFreq = 20;
//
//	// BatteryMeter
//	float MyVar::ConfigTable::BatteryMeterConfig::Ratio = 0.32591351983096153012896110238026f;
//	float MyVar::ConfigTable::BatteryMeterConfig::MaxVoltage = 8.2f;
//	float MyVar::ConfigTable::BatteryMeterConfig::MinVoltage = 7.55f;
//	uint16_t MyVar::ConfigTable::BatteryMeterConfig::UpdateFreq = 30000;
//
//	// Encoder
//	float MyVar::ConfigTable::EncoderConfig::UpdateFreq = 15;
//
//	// Lcd
//	uint8_t MyVar::ConfigTable::LcdConfig::Fps = 60;
//	uint16_t MyVar::ConfigTable::LcdConfig::LcdConfig::BgColor = 0;
//	uint16_t MyVar::ConfigTable::LcdConfig::TxtColor = -1;
////	uint16_t MyVar::ConfigTable::LcdConfig::HighPowerColor = 0x063F;
////	uint16_t MyVar::ConfigTable::LcdConfig::NormalPowerColor = 0x07E0;
////	uint16_t MyVar::ConfigTable::LcdConfig::LowPowerColor = 0xF800;
//	uint16_t MyVar::ConfigTable::LcdConfig::UpdateFreq = 20;
//
////#else
//
//	// Buzzer
//	int MyVar::ConfigTable::BuzzerConfig::WarningPitch = 70;
//	int MyVar::ConfigTable::BuzzerConfig::WarningTimes = 3;
//
//	// MagSen
//	float MyVar::ConfigTable::MagSenConfig::Kq = 0.6;
//	float MyVar::ConfigTable::MagSenConfig::Kr = 0.5;
//
//	// Servo
//	float MyVar::ConfigTable::ServoConfig::Reference = 0.0f;
//
//
//	float MyVar::ConfigTable::ServoConfig::NormalKp = 300.0f;
//	float MyVar::ConfigTable::ServoConfig::NormalKi = 0.0f;
//	float MyVar::ConfigTable::ServoConfig::NormalKd = 10.0f;
//
//	float MyVar::ConfigTable::ServoConfig::NormalWeightSD = 0.3589743f;
//	float MyVar::ConfigTable::ServoConfig::NormalWeightFD = 0.1794872f;
//	float MyVar::ConfigTable::ServoConfig::NormalWeightHD = 0.4615385f;
//
//
//	float MyVar::ConfigTable::ServoConfig::TurningKpA = 2600.0f;
//	float MyVar::ConfigTable::ServoConfig::TurningKpB = 0.0f;
//	float MyVar::ConfigTable::ServoConfig::TurningKi = 0.0f;
//	float MyVar::ConfigTable::ServoConfig::TurningKd = 40.0f;
//
//	float MyVar::ConfigTable::ServoConfig::TurningWeightSD = 0.1f;
//	float MyVar::ConfigTable::ServoConfig::TurningWeightFD = 0.325f;
//	float MyVar::ConfigTable::ServoConfig::TurningWeightHD = 0.525f;
//
//	float MyVar::ConfigTable::ServoConfig::TurningThresholdSdValue = 0.8f;
//	float MyVar::ConfigTable::ServoConfig::ForceTurningThresholdStartSdOutput = 0.5f;
//	float MyVar::ConfigTable::ServoConfig::ForceTurningThresholdStartFdOutput = 0.8f;
//	float MyVar::ConfigTable::ServoConfig::ForceTurningThresholdStopFdOutput = 0.6f;
//	float MyVar::ConfigTable::ServoConfig::ForceTurningThresholdStopHdOutput = 0.1f;
//
//
//	uint32_t MyVar::ConfigTable::ServoConfig::UpdateFreq = 7;
//
//	// Motor
//	float MyVar::ConfigTable::MotorConfig::Reference = 1200.0f;
//	float MyVar::ConfigTable::MotorConfig::Kp = 0.025f;
//	float MyVar::ConfigTable::MotorConfig::Ki = 0.0f;
//	float MyVar::ConfigTable::MotorConfig::Kd = 0.018f;
//
//	float MyVar::ConfigTable::MotorConfig::TurningSpeedRatio = 0.45f;
//	float MyVar::ConfigTable::MotorConfig::Turning90DegreeSpeedRatio = 0.3f;
//	float MyVar::ConfigTable::MotorConfig::MaxSpeedRatio = 1.8f;
//
//	uint32_t MyVar::ConfigTable::MotorConfig::TimeForReachingMaxSpeed = 2000;
//	int16_t MyVar::ConfigTable::MotorConfig::EmergencyStopThreshold = 0;
//
//	uint32_t MyVar::ConfigTable::MotorConfig::UpdateFreq = 20;
//
//	// VarMng
//	uint16_t MyVar::ConfigTable::VarMngConfig::UpdateFreq = 20;
//
//	// BatteryMeter
//	float MyVar::ConfigTable::BatteryMeterConfig::Ratio = 0.32591351983096153012896110238026f;
//	float MyVar::ConfigTable::BatteryMeterConfig::MaxVoltage = 8.2f;
//	float MyVar::ConfigTable::BatteryMeterConfig::MinVoltage = 7.6f;
//	uint16_t MyVar::ConfigTable::BatteryMeterConfig::UpdateFreq = 30000;
//
//	// Lcd
//	uint8_t MyVar::ConfigTable::LcdConfig::Fps = 60;
//	uint16_t MyVar::ConfigTable::LcdConfig::LcdConfig::BgColor = 0;
//	uint16_t MyVar::ConfigTable::LcdConfig::TxtColor = -1;
//	uint16_t MyVar::ConfigTable::LcdConfig::HighPowerColor = 0x063F;
//	uint16_t MyVar::ConfigTable::LcdConfig::NormalPowerColor = 0x07E0;
//	uint16_t MyVar::ConfigTable::LcdConfig::LowPowerColor = 0xF800;
//	uint16_t MyVar::ConfigTable::LcdConfig::UpdateFreq = 80;
//
////#endif
////
////#endif //IS_FRONT
