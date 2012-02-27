#include <iostream>
#include "ReadSonar.h"

class LedParser
{
	public:
		LedParser();
		~LedParser();
		void Green();
		void Red();
		void Yellow();
		void SendToLed();
		uint8_t Red;
		bool Green;
		bool Yellow;
	private:
		char C;
		ReadSonar* Sender;
}