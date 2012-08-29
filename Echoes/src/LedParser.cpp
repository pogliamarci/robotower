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

	if (request.editRed)
	{
		toggleRed(request.redNumOn);
	}
	if (request.editYellow)
	{
		toggleYellow(request.yellowOn, request.yellowBlinks);
	}
	if (request.editGreen)
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

void LedParser::toggleGreen(bool isOn, bool blinking)
{
	char buf[10];
	if (blinking && isOn)
	{
		sprintf(buf, "led G B\r\n");
	}
	else
	{
		sprintf(buf, "led G %c\r\n", isOn ? '1' : '0');
	}
	sender->sendStringCommand(buf, strlen(buf));
}

void LedParser::toggleRed(int num)
{
	char buf[10];
	char binaryNum[5];
	toBinaryString(binaryNum, num);
	sprintf(buf, "led R %s\r\n", binaryNum);
	sender->sendStringCommand(buf, strlen(buf));
}

void LedParser::toggleYellow(bool isOn, bool blinking)
{
	char buf[10];
	if (blinking && isOn)
	{
		sprintf(buf, "led Y B\r\n");
	}
	else
	{
		sprintf(buf, "led Y %s\r\n", isOn ? "1111" : "0000");
	}
	sender->sendStringCommand(buf, strlen(buf));
}

void LedParser::toBinaryString(char *binaryNum, int num)
{
	for (int i = 0; i < 4; i++)
	{
		binaryNum[0] = (i < num) ? '1' : '0';
	}
	binaryNum[4] = '\0';
}

