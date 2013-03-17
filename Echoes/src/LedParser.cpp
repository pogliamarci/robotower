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
}

/**
 * ROS callback for the led service. Forwards the request to the
 * board managing the leds.
 */
bool LedParser::redLedCallback(Echoes::FixedLed::Request& request,
		Echoes::FixedLed::Response& response)
{
	toggleRed(request.numOn);
	response.requestSuccessful = true;
	return true;
}

bool LedParser::greenLedCallback(Echoes::BlinkingLed::Request& request,
		Echoes::BlinkingLed::Response& response)
{
	toggleGreen(request.on, request.blinks);
	response.requestSuccessful = true;
	return true;
}

bool LedParser::yellowLedCallback(Echoes::BlinkingLed::Request& request,
		Echoes::BlinkingLed::Response& response)
{
	toggleYellow(request.on, request.blinks);
	response.requestSuccessful = true;
	return true;
}

bool LedParser::resetLedCallback(Echoes::ResetLed::Request& request,
		Echoes::ResetLed::Response& response)
{
	char buf[] = "reset\r\n";
	sender->sendStringCommand(buf, strlen(buf));
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
	char buf[15];
	char binaryNum[5];
	toBinaryString(binaryNum, num);
	sprintf(buf, "led R %s\r\n", binaryNum);
	sender->sendStringCommand(buf, strlen(buf));
}

void LedParser::toggleYellow(bool isOn, bool blinking)
{
	char buf[15];
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
		binaryNum[i] = (i < num) ? '1' : '0';
	}
	binaryNum[4] = '\0';
}

LedParser::~LedParser()
{
	Echoes::ResetLed::Request rqs;
	Echoes::ResetLed::Response res;
	resetLedCallback(rqs, res);
}

