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
		Dispatcher();
		void addProcesser(Processer* pr, string initial);
		void dispatch(string str);
		virtual ~Dispatcher();
	private:
		map<string, Processer*> processers;
};

#endif /* DISPATCHER_H_ */
