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
#define ALL_OFF 0x00
#define GREEN_ON 0x08
#define GREEN_OFF 0xF7
#define RED_OFF 0xF8
#define FIRST_YELLOW 16

//costruttore: inizializza le variabili, aggancia l'oggetto che invia dati al sonar
LedParser::LedParser(ReadSonar* read_sonar)
{
	int i;
	//inizializza lo stato dei led
	this->RedS=0;
	this->GreenS = false;
	for(i=0;i<4; i++) this->YellowS[i] = false;
	//inizializza lo stato binario
	this->C=ALL_OFF;
	//aggancia l'oggetto readsonar
	this->Sender = read_sonar;
}

//metodo per accendere la luce verde
void LedParser::Green(bool g)
{
	this->GreenS = g;
	if(GreenS==true) this->C = this->C|GREEN_ON;
	else this->C=this->C&GREEN_OFF;
}

//metodo per accendere le luci rosse
void LedParser::Red(char r)
{
	//aggiorna lo stato dei led rossi
	if (r <= NUMREDLED && r >= 1)
		this->RedS = r;
	else if (r > NUMREDLED)
		this->RedS = NUMREDLED;
	else this->RedS = 0;

	//aggiorna lo stato binario
	this->C = (this->C & RED_OFF) | RedS;

}

//metodo per accendere le luci gialle
void LedParser::Yellow(bool y[NUMREDLED])
{
	int i;
	char l=FIRST_YELLOW; //inizializza la maschera
	//aggiorna stato dei led gialli e stato binario bit a bit.
	for(i=0; i<NUMREDLED; i++) 
	{
		this->YellowS[i]=y[i];
		if(this->YellowS[i]==true) this->C=this->C|l;
		else this->C=this->C&(~l);
		l*=2; //shift a sinistra della maschera l 
	}
}

//metodo per inviare i comandi effettuati via zigbee al robot
void LedParser::SendToLed() 
{
	//manda il comando ai led come stringa di un carattere binario
	(this->Sender)->sendStringCommand(&C, 1);
}
