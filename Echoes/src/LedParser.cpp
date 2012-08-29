/*
 * RoboTower, Hi-CoRG based on ROS
 *
 * Copyright (C) 2012 Politecnico di Milano
 * Copyright (C) 2012 Marcello Pogliani, Davide Tateo
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
#include <cstdio>
#include "LedParser.h"

//costruttore: inizializza le variabili, aggancia l'oggetto che invia dati al sonar
LedParser::LedParser(SerialReader* read_sonar)
{
	this->sender = read_sonar;
	greenOn = false;
	yellowOn = 0;
	greenLedBlink = false;
	yellowLedsBlink = false;
	toggleYellow(true, true);
	toggleGreen(true, true);
}

/**
 * ROS callback for the led service. Forwards the request to the
 * board managing the leds.
 */
bool LedParser::ledCallback(Echoes::Led::Request& request,
		Echoes::Led::Response& response)
{

	if(request.editRed)
	{
		toggleRed(request.redNumOn);
	}
	if(request.editYellow)
	{
		toggleYellow(request.yellowOn, request.yellowBlinks);
	}
	if(request.editGreen)
	{
		toggleGreen(request.greenOn, request.greenBlinks);
	}
	response.requestSuccessful = true;
	return true;
}

bool LedParser::resetledCallback(Echoes::Led::Request& request,
		Echoes::Led::Response& response)
{
	greenLedBlink = false;
	yellowLedsBlink = false;

	toggleGreen(false, false);
	toggleYellow(false, false);
	toggleRed(0);

	response.requestSuccessful = true;
	return true;
}

void LedParser::sendCommands()
{
	char buf[20];
	if (greenLedBlink)
	{
		greenOn = !greenOn;
		sprintf(buf, "led G 0 %c\r\n", greenOn ? '1' : '0');
		sender->sendStringCommand(buf, strlen(buf));
	}
	if (yellowLedsBlink)
	{
		yellowOn = yellowOn == 3 ? 0 : yellowOn + 1;
		for (int i = 0; i < 4; i++)
		{
			char c = i + '0';
			sprintf(buf, "led Y %c %c\r\n", c,
					(i == yellowOn) ? '1' : '0');
			sender->sendStringCommand(buf, strlen(buf));
		}
	}
}

void LedParser::toggleGreen(bool isOn, bool blinking)
{
	greenLedBlink = blinking && isOn;
	if(!greenLedBlink)
	{
		char buf[10];
		sprintf(buf, "led G 0 %c\r\n", isOn ? '1' : '0');
		sender->sendStringCommand(buf, strlen(buf));
	}
}

void LedParser::toggleRed(int num)
{
	char buf[10];
	for (int i = 0; i < 4; i++)
	{
		sprintf(buf, "led R %c %c\r\n", (char) i + '0',
				i < num ? '1' : '0');
		sender->sendStringCommand(buf, strlen(buf));
	}
}

void LedParser::toggleYellow(bool isOn, bool blinking) {
	yellowLedsBlink = blinking && isOn;
	if(!yellowLedsBlink)
	{
		for (char c = '0'; c < '1'; c++)
		{
			char buf[10];
			sprintf(buf, "led Y %c %c\r\n", c, isOn ? '1' : '0');
			sender->sendStringCommand(buf, strlen(buf));
		}
	}
}
