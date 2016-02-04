/*
 * MyFlash.h
 *
 * Author: Peter Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include <cassert>
#include <libbase/helper.h>
#include <libbase/k60/flash.h>

using namespace libbase::k60;

class MyFlash : Flash
{

public:

	MyFlash(void);

	void eraseAll(void);
	static void writeConfig(void);

private:

	size_t			m_sizeNeeded;
	static MyFlash	*m_instance;

	void readConfig(void);

};
