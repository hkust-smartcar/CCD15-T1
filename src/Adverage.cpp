/*
 * Adverage.cpp
 *
 *  Created on: 12 Jan, 2016
 *      Author: lincoln
 */

#include "Adverage.h"

Adverage::Adverage(const int total, int window) {
	for(int i=0;i<total;i++)
		my_queue.push(0);
	Adverage::window = window;
}

Adverage::~Adverage() {
}

void Adverage::store(){


}

void Adverage::moving_adverage(){
//	int temp = 0;
//	int temp_data[128];
//	for(int i =0;i<128;i++){
//		temp = 0;
//		if(i==0 || i == 127){
//
//		}
//		else{
//			for(int a=i-1;(a-i+1)<window;a++){
//				temp += my_queue[a];
//			}
//			temp_data[i] = (uint32_t)(temp/(float)window);
//		}
//	}
//	for(int i=0;i<128;i++){
//		my_queue[i] = temp_data[i];
//	}
}
