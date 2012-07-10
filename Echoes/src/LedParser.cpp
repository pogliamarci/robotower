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
LedParser::LedParser(ReadSonar* read_sonar)
{
	this->sender = read_sonar;
}

/**
 * ROS callback for the led service. Forwards the request to the
 * board managing the leds.
 */
bool LedParser::ledCallback(Echoes::Led::Request& request, Echoes::Led::Response& response)
{

	if (request.editGreen == true)
	{
		sprintf(buf, "led G 1 %c", request.greenIsOn ? '1' : '0');
		sendCmd(buf);
	}
	if (request.editRed == true)
	{
		for(int i = 0; i < 4; i++)
		{
			sprintf(buf, "led R %c %c", (char) i + '0',
					i < request.redNumOn ? '1' : '0');
			sendCmd(buf);
		}
	}
	if (request.editYellow == true)
	{
		for (int i = 0; i < 4; i++)
		{
			sprintf(buf, "led Y %c %c", (char) i + '0',
					i < request.yellowIsOn[i] ? '1' : '0');
			sendCmd(buf);
		}
	}
	response.requestSuccessful = true;
	return true;
}

void LedParser::sendCmd(char* buf)
{
	sender->sendStringCommand(buf, strlen(buf));
}
