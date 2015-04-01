#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/encoder.h>
#include <libsc/dir_motor.h>
#include <libsc/st7735r.h>
#include <libsc/lcd_console.h>
#include <libutil/string.h>
#include <libsc/trs_d05.h>
#include <sstream>
#include <string>
#include <libsc/simple_buzzer.h>
#include "libbase/log.h"
#include <libsc/k60/ftdi_ft232r.h>
#include <libsc/k60/led.h>
#include <libsc/tsl1401cl.h>
#include <libbase/k60/gpio.h>

using namespace libsc::k60;
using namespace libsc;
using namespace libbase::k60;


namespace libbase
{
namespace k60
{

Mcg::Config Mcg::GetMcgConfig()
{
	Mcg::Config config0;
	// Do NOT change
	config0.external_oscillator_khz = 50000;
	// Set MCU clock to 100MHz
	config0.core_clock_khz = 100000;
	return config0;
}

}
}

// Enter point of the program, i.e., your program starts here
int main()
{
	// Initialize the system
	System::Init();

	St7735r::Config config1;
	config1.is_revert = false;
	St7735r lcd(config1);
	//	LcdConsole console(&lcd);

	Tsl1401cl ccd(0);

	Gpo::Config lincoln;
	lincoln.pin = Pin::Name::kPtc11;
	Gpo howard(lincoln);

	lcd.Clear(0);
	Timer::TimerInt t = 0, pt = t;

	ccd.StartSample();

	while(true){
		t = System::Time();
		if(t - pt >= 400)
		{
			pt = t;

			howard.Set(1);
			ccd.StartSample();
//			System::DelayMs(1);
			while (!ccd.SampleProcess()){}

			uint32_t sum = 0;

			std::array<uint16_t, Tsl1401cl::kSensorW> Data = ccd.GetData();

			for(int i = 0; i < Tsl1401cl::kSensorW; i++){
				Data[i] = Data[i] * 80 / 65535;
				sum += Data[i];
			}

			uint16_t average = sum / Tsl1401cl::kSensorW;
			if(average > 72)
				average = 72;
			else if(average < 35)
				average = 35;
			else
				average = average + 3;

			St7735r::Rect rect_1, rect_2, rect_3, rect_4;
			for(int i = 0; i<Tsl1401cl::kSensorW; i++){
				rect_1.x = i;
				rect_1.y = 0;
				rect_1.w = 1;
				rect_1.h = Data[i];
				rect_2.x = i;
				rect_2.y = Data[i];
				rect_2.w = 1;
				rect_2.h = 80 - Data[i];
				lcd.SetRegion(rect_1);
				lcd.FillColor(~0);
				lcd.SetRegion(rect_2);
				lcd.FillColor(0);
			}

			for(int i=0; i<Tsl1401cl::kSensorW; i++){
				if(Data[i] < average)
					Data[i] = 0;
				else
					Data[i] = 60;
			}

			for(int i = 0; i<Tsl1401cl::kSensorW; i++){
				rect_3.x = i;
				rect_3.y = 90;
				rect_3.w = 1;
				rect_3.h = Data[i];
				rect_4.x = i;
				rect_4.y = 90 + Data[i];
				rect_4.w = 1;
				rect_4.h = 60 - Data[i];
				lcd.SetRegion(rect_3);
				lcd.FillColor(~0);
				lcd.SetRegion(rect_4);
				lcd.FillColor(0);
			}

			howard.Set(0);
		}
		//		System::DelayMs(20);
	}
}

// LCD from right to left is 0 - 127;
