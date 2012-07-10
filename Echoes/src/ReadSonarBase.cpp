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

#include "ReadSonarBase.h"

#include <iostream>

#include <string>
using namespace std;

const char ReadSonarBase::response_ok_header[]="+OK ";
const char ReadSonarBase::response_err_header[]="-ERR";
const char ReadSonarBase::response_dbg_header[]="%DBG";
const char ReadSonarBase::response_meas_header[]="+OK ";


const char* ReadSonarDeviceException::what() const throw()
{
  return "Sonar port open error!";
}

ReadSonarBase::ReadSonarBase(float to_meter)
throw (ReadSonarDeviceException)
{
	
	pack_n=0;	
	measure=NULL;
	buffer=NULL;
	tmp_buf=NULL;
	// this->to_meter=to_meter;
	
	measure=new float[ReadSonarBase::n_sonar];
	tmp_buf=new char[ReadSonarBase::max_buf_tmp];
	for(unsigned int i=0;i<ReadSonarBase::n_sonar;i++)
	{
		measure[i]=-1;
	}
	
}

ReadSonarBase::~ReadSonarBase()
{
	if(measure)delete [] measure;
	if(tmp_buf)delete [] tmp_buf;
}

unsigned int ReadSonarBase::getLastPackNum()
{
	return pack_n;
}

string ReadSonarBase::getLine()
{
	if(buffer->getLineCount()<=0) return "Error";
	int len=buffer->removeLine(tmp_buf,max_buf_tmp);
	if(len<=0)return "Error";
	if(tmp_buf[len-1]=='\n')tmp_buf[len-1]='\0';
	return tmp_buf;
}

unsigned int ReadSonarBase::getLineToParseNum()
{
	return buffer->getLineCount();
}
