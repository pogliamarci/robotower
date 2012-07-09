/*
 * Processer.h
 *
 *  Created on: 9 Jul 2012
 *      Author: marcello
 */

#ifndef PROCESSER_H_
#define PROCESSER_H_

#include <string>

using namespace std;

class Processer
{
	public:
		virtual void process(string str) = 0;
		virtual ~Processer() {}
};

#endif /* PROCESSER_H_ */
