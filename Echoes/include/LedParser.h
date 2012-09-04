/*
 * RoboTower, Hi-CoRG based on ROS
 *
 * Copyright (C) 2012 Politecnico di Milano
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

#ifndef LED_PARSER_H
#define LED_PARSER_H

#include <iostream>
#include "SerialCommunication.h"
#include "ros/ros.h"
#include "Echoes/BlinkingLed.h"
#include "Echoes/FixedLed.h"
#include "Echoes/ResetLed.h"

class LedParser
{
private:
	SerialReader* sender;

public:
	LedParser(SerialReader* read_sonar);
	bool redLedCallback(Echoes::FixedLed::Request& request,
			Echoes::FixedLed::Response& response);
	bool greenLedCallback(Echoes::BlinkingLed::Request& request,
				Echoes::BlinkingLed::Response& response);
	bool yellowLedCallback(Echoes::BlinkingLed::Request& request,
				Echoes::BlinkingLed::Response& response);
	bool resetLedCallback(Echoes::ResetLed::Request& request,
			Echoes::ResetLed::Response& response);
	void sendCommands();
	~LedParser();

private:
	void toggleGreen(bool isOn, bool blinking);
	void toggleRed(int num);
	void toggleYellow(bool isOn, bool blinking);
	void toBinaryString(char *binaryNum, int num);
};

#endif
