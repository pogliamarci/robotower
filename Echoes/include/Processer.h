/*
 * Processer.h
 *
 *  Created on: 9 Jul 2012
 *      Author: marcello
 */

#ifndef PROCESSER_H_
#define PROCESSER_H_

#include <string>
#include <vector>

using namespace std;

class Processer
{
	public:
		virtual void process(string str) = 0;
		virtual ~Processer() {}
	protected:
		void tokenize(const std::string& str,
				                std::vector<std::string>& tokens,
				                const std::string& delimiters);
};

#endif /* PROCESSER_H_ */
