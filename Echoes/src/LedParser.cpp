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
	redLedNumber = 0;
}

/**
 * ROS callback for the led service. Forwards the request to the
 * board managing the leds.
 */
bool LedParser::ledCallback(Echoes::Led::Request& request,
		Echoes::Led::Response& response)
{
	if (request.editRed)
	{
		redLedNumber = request.redNumOn;
	}
	if (request.editGreen)
	{
		greenLedBlink = request.greenBlinks;
	}
	if (request.editYellow)
	{
		yellowLedsBlink = request.yellowBlinks;
	}

	bool green = request.editGreen && !greenLedBlink;
	bool red = request.editRed;
	bool yellow = request.editYellow && !yellowLedsBlink;
	sendOnOffCommands(green, red, yellow, true);

	response.requestSuccessful = true;
	return true;
}

bool LedParser::resetledCallback(Echoes::Led::Request& request,
		Echoes::Led::Response& response)
{
	greenLedBlink = false;
	yellowLedsBlink = false;
	redLedNumber = 0;
	sendOnOffCommands(true, true, true, false);
	response.requestSuccessful = true;
	return true;
}

void LedParser::sendCommands()
{
	char buf[10];
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
			sprintf(buf, "led Y %c %c\r\n", (char) i + '0',
					(i == yellowOn) ? '1' : '0');
			sender->sendStringCommand(buf, strlen(buf));
		}
	}
}

void LedParser::sendOnOffCommands(bool green, bool red, bool yellow, bool isOn)
{
	char buf[10];
	if (red)
	{
		for (int i = 0; i < 4; i++)
		{
			sprintf(buf, "led R %c %c\r\n", (char) i + '0',
					i < redLedNumber ? '1' : '0');
			sender->sendStringCommand(buf, strlen(buf));
		}
	}
	if (green)
	{
		sprintf(buf, "led G 0 %c\r\n", isOn ? '1' : '0');
		sender->sendStringCommand(buf, strlen(buf));
	}
	if (yellow)
	{
		for (int i = 0; i < 4; i++)
		{
			sprintf(buf, "led Y %c %c\r\n", (char) i + '0', isOn ? '1' : '0');
			sender->sendStringCommand(buf, strlen(buf));
		}
	}
}

