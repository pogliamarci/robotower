#include <iostream>
#include "ReadSonar.h"

class LedParser
{
	public:
		LedParser();
		~LedParser();
		void Green(bool g);
		void Red(char r);
		void Yellow(bool y[4]);
		void SendToLed();
		char Red;
		bool Green;
		bool Yellow;
	private:
		char C;
		ReadSonar* Sender;
}