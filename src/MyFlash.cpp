/*
 * MyFlash.cpp
 *
 * Author: Peter Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <libbase/k60/flash.h>
#include "MyFlash.h"
#include "MyVar.h"

using namespace libbase::k60;

MyFlash			*MyFlash::m_instance = nullptr;

Flash::Config getFlashConfig(void)
{
	Flash::Config config;
	return config;
}

MyFlash::MyFlash(void)
:
	Flash(getFlashConfig()),
	m_sizeNeeded(0)
{
	if (!m_instance)
		m_instance = this;

//	m_sizeNeeded = (uint32_t)&MyVar::ConfigTable::LcdConfig::UpdateFreq + sizeof(MyVar::ConfigTable::LcdConfig::UpdateFreq) -
//						(uint32_t)&MyVar::ConfigTable::MagSenConfig::Kq;
	m_sizeNeeded = sizeof(MyVar::ConfigTable::testing::test);

	readConfig();
}

void MyFlash::eraseAll(void)
{
	EraseSector(GetStartAddr());
}

void MyFlash::readConfig(void)
{
	if (*((uint8_t *)(GetStartAddr() + m_sizeNeeded - 1)) == 0xFF || *((uint8_t *)GetStartAddr()) == 0xFF || *((uint8_t *)(GetStartAddr() + m_sizeNeeded)) != 0xFF)
		return ;

	assert(Read(&MyVar::ConfigTable::testing::test, m_sizeNeeded) == FlashStatus::kSuccess);
}

void MyFlash::writeConfig(void)
{
	m_instance->Write(&MyVar::ConfigTable::testing::test, m_instance->m_sizeNeeded);
}
