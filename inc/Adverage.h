/*
 * Adverage.h
 *
 *  Created on: 12 Jan, 2016
 *      Author: lincoln
 */

#ifndef SRC_ADVERAGE_H_
#define SRC_ADVERAGE_H_
#include "stdint.h"
#include <queue>

class Adverage {
private:
//	int * arr;
	std::queue<int> my_queue;
	int window;
public:
	Adverage(const int total, int window);
	~Adverage();
	void moving_adverage();
	void store();
};

#endif /* SRC_ADVERAGE_H_ */
