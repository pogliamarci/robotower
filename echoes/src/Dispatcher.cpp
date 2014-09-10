#include "Dispatcher.h"
#include <iostream>

void Dispatcher::addProcesser(Processer* pr, string initial)
{
	processers[initial] = pr;
}

void Dispatcher::dispatch(string str)
{
	for(map<string, Processer*>::iterator ii=processers.begin();
			ii!=processers.end(); ++ii)
	{
		if(str.find((*ii).first) == 0) /* the key is contained at the beginning of the string to process */
		{
			(*ii).second->process(str.substr((*ii).first.size()+1));
			return;
		}
	}
	// default dispatching: print str to stderr
	cerr << str << endl;
}
