#include <iostream>
#include "LedParser.h"

LedParser::LedParser()
{
	this.Red=0;
	this.Green=FALSE;
	this.Yellow[4]=FALSE;
	this.C=0x00;
}

LedParser::Green(bool g)
{
	this.Green=g;
	if(Green=TRUE) this.C=this.C|0x08;
	else this.C=this.C&0xF7;
}

void LedParser::Red(uint8_t r)
{
	this.Red=r;
	this.C=this.C&(8+Red);
}

void LedParser::Yellow(bool y[4])
{
	int i;
	char l=32;
	for(i=0; i<4; i++) 
	{
		this.Yellow[i]=y[i];
		if(this.Yellow[i]=TRUE) this.C=this.C|l;
		else this.C=this.C&(!l);
		l*2;
	}
}

void LedParser::SendToLed() 
{
	this.Sender->sendStringCommand(&C, 1);
}