/*
 * Dispatcher.h
 *
 *  Created on: 9 Jul 2012
 *      Author: marcello
 */

#ifndef DISPATCHER_H_
#define DISPATCHER_H_

#include <string>
#include <map>
#include"Processer.h"

using namespace std;

class Dispatcher
{
	public:
		void addProcesser(Processer* pr, string initial);
		void dispatch(string str);
	private:
		map<string, Processer*> processers;
};

#endif /* DISPATCHER_H_ */
