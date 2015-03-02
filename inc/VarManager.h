/*
 * VarManager.h
 *
 * Author: PeterLau
 * Version: 2.8.0
 *
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#ifndef INC_VARMANAGER_H_

// TODO: enable following preprocessor command
//#ifdef LIBSC_USE_UART
#define INC_VARMANAGER_H_

#include <libsc/k60/system.h>
#include <libsc/k60/ftdi_ft232r.h>
#include <libbase/k60/sys_tick.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <typeinfo>
#include <string.h>
#include <cxxabi.h>

using namespace libsc::k60;
using namespace libbase::k60;

class VarManager
{
public:

	class ObjMng
	{
	public:

		explicit ObjMng(void *pObj, Byte len, const std::string &typeName, const std::string &objName)
		:
			obj(pObj),
			len(len),
			typeName(typeName),
			varName(objName)
		{}

		~ObjMng() {};

		void						*obj;
		Byte						len;
		std::string					typeName;
		std::string					varName;
	};

	class TypeId
	{
	public:

		static void Init()
		{
			if (!m_instance)
				m_instance = new TypeId;
		}

		static std::string getTypeId(uint8_t o) { return "unsigned char"; }
		static std::string getTypeId(int8_t o) { return "signed char"; }
		static std::string getTypeId(uint16_t o) { return "unsigned short"; }
		static std::string getTypeId(int16_t o) { return "short"; }
		static std::string getTypeId(uint32_t o) { return "unsigned int"; }
		static std::string getTypeId(int32_t o) { return "int"; }
		static std::string getTypeId(float o) { return "float"; }
		template<typename T>
		static std::string getTypeId(T o) { return "wtf?"; }

	private:

		static TypeId *m_instance;
	};

	explicit VarManager(void);
	~VarManager(void);

	void Init(void);
	void Init(const JyMcuBt106::OnReceiveListener &oriListener);
	void UnInit(void);

	template<typename ObjType>
	void addSharedVar(ObjType *sharedObj, std::string s)
	{
		if (!isStarted)
		{
			ObjMng newObj(sharedObj, sizeof(sharedObj), TypeId::getTypeId(*sharedObj), s);
			sharedObjMng.push_back(newObj);
		}
	}

	template<typename ObjType>
	void addWatchedVar(ObjType *watchedObj, std::string s)
	{
		if (!isStarted)
		{
			ObjMng newObj(watchedObj, sizeof(*watchedObj), TypeId::getTypeId(*watchedObj), s);
			watchedObjMng.push_back(newObj);
		}
	}

	void sendWatchData(void);

private:

	JyMcuBt106						m_uart;

	JyMcuBt106::OnReceiveListener	m_origin_listener;

	std::vector<ObjMng>				sharedObjMng;
	std::vector<ObjMng>				watchedObjMng;

	bool							isStarted;
	const Byte						rx_threshold;

	std::vector<Byte>				rx_buffer;

	static void listener(const Byte *bytes, const size_t size);

	SysTick::Config getTimerConfig(void);
	JyMcuBt106::Config get106UartConfig(const uint8_t id);
	FtdiFt232r::Config get232UartConfig(const uint8_t id);

	void sendWatchedVarInfo(void);
	void sendSharedVarInfo(void);

};

//#endif /* LIBSC_USE_UART */
#endif /* INC_VARMANAGER_H_ */