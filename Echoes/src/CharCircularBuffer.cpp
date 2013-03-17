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

#include "CharCircularBuffer.h"

#if (defined TEST_VERSION) || (defined  DEBUG)
#include <sstream>
#include <iostream>
#include <string>

#endif
CharCircularBuffer::CharCircularBuffer(unsigned int n, char end_line_char)
{
	buf = new char[n + 1];
	this->n = n + 1;
	start = 0;
	end = 0;
	lineCount = 0;
	this->end_line_chars = new char[1];
	this->end_line_chars[0] = end_line_char;
	this->end_line_chars_num = 1;
}

CharCircularBuffer::CharCircularBuffer(unsigned int n, char * end_line_chars)
{
	buf = new char[n + 1];
	this->n = n + 1;
	start = 0;
	end = 0;
	lineCount = 0;
	this->end_line_chars_num = strlen(end_line_chars);
	this->end_line_chars = new char[this->end_line_chars_num + 1];
	strcpy(this->end_line_chars, end_line_chars);
}

CharCircularBuffer::~CharCircularBuffer()
{
	delete[] buf;
	delete[] end_line_chars;
}

unsigned int CharCircularBuffer::getLineCount()
{
	return lineCount;
}

//ret n char added (0: error, 1 ok)
int CharCircularBuffer::addChar(char src)
{
	if (isFull())
		return 0;
	buf[end] = src;
	end = inc(end);
	if (isEndLine(src))
		lineCount++;
	return 1;
}

int CharCircularBuffer::isEndLine(char c)
{
	for (int i = 0; i < end_line_chars_num; i++)
	{
		if (c == end_line_chars[i])
			return 1;
	}
	return 0;
}

//ret n char removed (0: buffer empty)
int CharCircularBuffer::removeChar(char *dest)
{
	if (isEmpty())
		return 0;
	*dest = buf[start];
	start = inc(start);
	if (isEndLine(*dest))
		lineCount--;
	return 1;
}

//ret n char added (0: error, 1 ok)
int CharCircularBuffer::addNChar(char *src, unsigned int nx)
{
	unsigned int disp = n - getCount() - 1; //#posti disponibili
	nx = (disp > nx ? nx : disp);
	for (unsigned int i = 0; i < nx; i++)
	{
		if (isEndLine(src[i]))
			lineCount++;
		buf[end] = src[i];
		end = inc(end);
	}
	return nx;
}

//ret n char removed (0: buffer empty, 0-n)
int CharCircularBuffer::removeNChar(char *dest, unsigned int nx)
{
	unsigned int disp = getCount(); //#posti disponibili
	nx = (disp > nx ? nx : disp);
	for (unsigned int i = 0; i < nx; i++)
	{
		if (isEndLine(buf[start]))
			lineCount--;
		dest[i] = buf[start];
		start = inc(start);
	}
	return nx;
}

//ret nchar readed, (max maxn char), terminate string with \0 (instead of \n or in maxn position)
int CharCircularBuffer::removeLine(char *dest, unsigned int maxn)
{
	unsigned int c = 0;
	if (lineCount == 0)
		return 0;
	while ((!isEmpty()) && (!isEndLine(buf[start])) && (c < maxn - 1))
	{
		dest[c] = buf[start];
		start = inc(start);
		c++;
	}
	dest[c] = '\0';
	if (!isEmpty() && isEndLine(buf[start]))
	{
		lineCount--;
		start = inc(start);
	}
	return c;
}

#if (defined TEST_VERSION) || (defined  DEBUG)

std::string CharCircularBuffer::getStringStatus()
{
	std::ostringstream ss;
	char str[n];
	unsigned int x;
	unsigned int i=0;
	x=start;
	for (i=0;i<getCount();i++)
	{
		//std::cout<<"I:"<<i<<std::endl;
		str[i]=buf[x];
		x=inc(x);
	}
	str[i]='\0';
	ss<<"#:"<<getCount()<<" s="<<start<<" e="<<end<<" l="<<getLineCount()<<" E?"<<isEmpty()<<" F?"<<isFull()<<" buf=["<<str<<"]\n";

	return ss.str();

}
#endif
