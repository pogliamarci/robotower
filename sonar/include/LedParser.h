#include <iostream>
#include "ReadSonar.h"

class LedParser
{
	public:
		LedParser(ReadSonar* read_sonar);
		void Green(bool g);
		void Red(char r);
		void Yellow(bool y[4]);
		void SendToLed();
		char RedS;
		bool GreenS;
		bool YellowS[4];
	private:
		char C;
		ReadSonar* Sender;
};
