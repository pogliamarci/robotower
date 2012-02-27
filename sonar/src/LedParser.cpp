#include <iostream>
#include <cstdio>
#include "LedParser.h"

#define NUMREDLED 4

LedParser::LedParser(ReadSonar* read_sonar)
{
    int i;
	this->RedS=0;
	this->GreenS = false;
	for(i=0;i<4; i++)
	    this->YellowS[i] = false;
	this->C=0x00;
	this->Sender = read_sonar;
}

void LedParser::Green(bool g)
{
	this->GreenS = g;
	if(GreenS==true) this->C = this->C|0x08;
	else this->C=this->C&0xF7;
}

void LedParser::Red(char r)
{
    if (r <= NUMREDLED && r >= 1)
        this->RedS = r;
    else if (r > NUMREDLED)
        this->RedS = NUMREDLED;
    else this->RedS = 0;

    this->C = (this->C & 0xF8) | RedS;
}

void LedParser::Yellow(bool y[4])
{
	int i;
	char l=16;
	for(i=0; i<4; i++) 
	{
		this->YellowS[i]=y[i];
		if(this->YellowS[i]==true) this->C=this->C|l;
		else this->C=this->C&(~l);
		l*=2;
	}
}

void LedParser::SendToLed() 
{
    printf("numero %d\n", C);
	(this->Sender)->sendStringCommand(&C, 1);
}
