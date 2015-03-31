/*
 * MyVarManager.cpp
 *
 * Author: PeterLau
 * Version: 2.8.0
 *
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cstdlib>
#include <functional>
#include <utility>


#include <libsc/system.h>
#include <libbase/k60/sys_tick.h>
#include <libsc/k60/jy_mcu_bt_106.h>

#include "MyVarManager.h"

// TODO: enable following preprocessor command
//#ifdef LIBSC_USE_UART

using namespace libbase::k60;
using namespace libsc;
using namespace libsc::k60;
using namespace std;

MyVarManager *m_pd_instance;

MyVarManager::TypeId *MyVarManager::TypeId::m_instance = nullptr;

JyMcuBt106::Config MyVarManager::get106UartConfig(const uint8_t id)
{
	JyMcuBt106::Config config;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.rx_irq_threshold = rx_threshold;
	config.is_rx_irq_threshold_percentage = false;
	config.tx_buf_size = 50;
	config.rx_isr = &listener;
	return config;
}

FtdiFt232r::Config MyVarManager::get232UartConfig(const uint8_t id)
{
	FtdiFt232r::Config config;
	config.id = id;
	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	config.rx_irq_threshold = rx_threshold;
	config.is_rx_irq_threshold_percentage = false;
	config.tx_buf_size = 50;
	return config;
}

MyVarManager::MyVarManager(void)
:
	rx_threshold(7),
	m_uart(get106UartConfig(0)),
	isStarted(false)
{
	m_pd_instance = this;
	System::Init();
	TypeId::Init();
}

MyVarManager::~MyVarManager()
{
	sharedObjMng.clear();
	watchedObjMng.clear();
}

bool MyVarManager::listener(const std::vector<Byte> &bytes)
{
	m_pd_instance->rx_buffer.insert(m_pd_instance->rx_buffer.end(), bytes.begin(), bytes.end());

	if (m_pd_instance->rx_buffer.size() < m_pd_instance->rx_threshold)
		return true;

	if (m_pd_instance->rx_buffer.size() == m_pd_instance->rx_threshold)
	{
		if (m_pd_instance->rx_buffer.at(1))
		{
			switch (m_pd_instance->rx_buffer.at(0))
			{
			case 's':
				m_pd_instance->isStarted = true;
				break;

			case 'e':
				m_pd_instance->isStarted = false;
				break;

			case 'w':
				m_pd_instance->sendWatchedVarInfo();
				break;

			case 'h':
				m_pd_instance->sendSharedVarInfo();
				break;

			case 'c':
				// TODO: change variable here
				break;
			}
		}
		else
			m_pd_instance->m_origin_listener(m_pd_instance->rx_buffer);
	}

	m_pd_instance->rx_buffer.clear();
	return true;
}

void MyVarManager::sendWatchData(void)
{
	if (isStarted)
		for (Byte i = 0; i < watchedObjMng.size(); i++)
			m_uart.SendBuffer((Byte *)((ObjMng)watchedObjMng.at(i)).obj, ((ObjMng)watchedObjMng.at(i)).len);
}

void MyVarManager::sendWatchedVarInfo(void)
{
	m_uart.SendBuffer((Byte *)",", 1);
	Byte n = watchedObjMng.size();
	m_uart.SendBuffer((Byte *)&n, 1);
	for (Byte i = 0; i < watchedObjMng.size(); i++)
	{
		ObjMng temp = watchedObjMng.at(i);
		m_uart.SendBuffer((Byte *)temp.typeName.data(), temp.typeName.size() + 1);
		m_uart.SendBuffer((Byte *)temp.varName.data(), temp.varName.size() + 1);
		m_uart.SendBuffer((Byte *)",", 1);
	}
	m_uart.SendBuffer((Byte *)"end", 3);
}

void MyVarManager::sendSharedVarInfo(void)
{
	m_uart.SendBuffer((Byte *)".", 1);
	Byte n = sharedObjMng.size();
	m_uart.SendBuffer((Byte *)&n, 1);
	for (Byte i = 0; i < sharedObjMng.size(); i++)
	{
		ObjMng temp = sharedObjMng.at(i);
		m_uart.SendBuffer((Byte *)temp.typeName.data(), temp.typeName.size() + 1);
		m_uart.SendBuffer((Byte *)temp.varName.data(), temp.varName.size() + 1);
		m_uart.SendBuffer((Byte *)temp.obj, temp.len);
		m_uart.SendBuffer((Byte *)".", 1);
	}
	m_uart.SendBuffer((Byte *)"end", 3);
}

void MyVarManager::Init(void)
{
	if (!isStarted)
	{
		if (!m_origin_listener)
			m_origin_listener = nullptr;
	}
}

void MyVarManager::Init(const OnReceiveListener &oriListener)
{
	if (!isStarted)
	{
		m_origin_listener = oriListener;
	}
}

void MyVarManager::UnInit(void)
{
	m_origin_listener = nullptr;
}

//#endif /* LIBSC_USE_UART */