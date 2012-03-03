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

#include <iostream>
#include "ReadSonar.h"

#define NUMREDLED 4

//classe per la gestione a basso livello dei led
class LedParser
{
	public:
		//costruttore
		LedParser(ReadSonar* read_sonar);
		//metodo per accendere i led verdi
		void Green(bool g);
		//metodo per accendere i led rossi
		void Red(char r);
		//metodo per accendere i led gialli
		void Yellow(bool y[NUMREDLED]);
		//metodo per inviare i dati ai led
		void SendToLed();
		//stato dei led rossi
		char RedS;
		//stato dei led verdi
		bool GreenS;
		//stato dei led gialli
		bool YellowS[NUMREDLED];
	private:
		//stato dei let binario
		char C;
		//oggetto per comunicare via zigbee
		ReadSonar* Sender;
};
