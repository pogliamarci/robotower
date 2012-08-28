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

#ifndef CharCircularBuffer_h_
#define CharCircularBuffer_h_

#include <cstdio>
#include <string.h>
#include <string>


class CharCircularBuffer
{
	char * buf;
	unsigned int start,end;
	
	char * end_line_chars;
	int end_line_chars_num;
	
	unsigned int n;
	unsigned int lineCount;
	
	inline unsigned int inc(unsigned int v)
	{
		return (v+1==n?0:v+1);
	};
	inline unsigned int dec(unsigned int v)
	{
		return (v==0?n-1:v-1);
	};
	
	int isEndLine(char c);
	
	public:
		CharCircularBuffer(unsigned int n,char end_line_char='\n');
		CharCircularBuffer(unsigned int n,char * end_line_chars);
		~CharCircularBuffer();

		inline unsigned int getCount()
		{
				return (end>=start?end-start:n-start+end);
		}
		
		unsigned int getLineCount();
		
		int addChar(char src); //ret n char added (0: error, 1 ok)
		int removeChar(char *dest);//ret n char removed (0: buffer empty)

		inline void reset()
		{
			start=end=0;
		};

		int addNChar(char *src,unsigned int n);//ret n char added (0: error, 1 ok)
		int removeNChar(char *dest,unsigned int n);//ret n char removed (0: buffer empty, 0-n)

		int removeLine(char *dest,unsigned int maxn);//ret nchar readed, max maxn
		
		inline bool isFull()
		{
			return inc(end)==start;
		};
		inline bool isEmpty()
		{
			return start==end;
		};

		#if (defined TEST_VERSION) || (defined  DEBUG)
		std::string getStringStatus();
		#endif

};

#endif
