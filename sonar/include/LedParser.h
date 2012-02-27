/*
 * RoboTower, Hi-CoRG based on ROS
 *
 *
 * Copyright (C) 2011 Marcello Pogliani, Davide Tateo
 * Versione 1.0
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

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
		char RedS;
		bool GreenS;
		bool YellowS;
	private:
		char C;
		ReadSonar* Sender;
}
