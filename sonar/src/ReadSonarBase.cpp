/*
 * RoboTower, Hi-CoRG based on ROS
 *
 *
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

#include "ReadSonarBase.h"

#include <iostream>

const char ReadSonarBase::response_ok_header[]="+OK ";
const char ReadSonarBase::response_err_header[]="-ERR";
const char ReadSonarBase::response_dbg_header[]="%DBG";
const char ReadSonarBase::response_meas_header[]="+OK ";


const char* ReadSonarDeviceException::what() const throw(){
  return "Sonar port open error!";
}

ReadSonarBase::ReadSonarBase(float to_meter)
		throw (ReadSonarDeviceException){
	
	pack_n=0;	
	measure=NULL;
	buffer=NULL;
	tmp_buf=NULL;
	this->to_meter=to_meter;
	
	measure=new float[ReadSonarBase::n_sonar];
	tmp_buf=new char[ReadSonarBase::max_buf_tmp];
	for(unsigned int i=0;i<ReadSonarBase::n_sonar;i++){
		measure[i]=-1;
	}
	
}

ReadSonarBase::~ReadSonarBase(){
	if(measure)delete [] measure;
	if(tmp_buf)delete [] tmp_buf;
}

unsigned int ReadSonarBase::getLastPackNum(){
	return pack_n;
}

float ReadSonarBase::getMeasure(unsigned int index){
	if(!measure)return -1;
	if(index<0)return -1;
	if(index>=n_sonar)return -1;
	return measure[index];
}

int ReadSonarBase::parseLine(){
	int parsed_type = ReadSonarBase::parse_err;
	if(buffer->getLineCount()<=0)return parsed_type;
	
	int len=buffer->removeLine(tmp_buf,max_buf_tmp);
	if(len<=0)return parsed_type;
	if(tmp_buf[len-1]=='\n')tmp_buf[len-1]='\0';
	
	if(strncmp(tmp_buf,ReadSonarBase::response_err_header,ReadSonarBase::response_header_len)==0){
		parsed_type = ReadSonarBase::parse_response_err;
	}
	else if(strncmp(tmp_buf,ReadSonarBase::response_dbg_header,ReadSonarBase::response_header_len)==0){
		parsed_type = ReadSonarBase::parse_dbg;
	}
	else if(strncmp(tmp_buf,ReadSonarBase::response_meas_header,ReadSonarBase::response_header_len)==0){
		std::vector<std::string> tokens;
		tokenize(std::string(tmp_buf),tokens,",");
		if(tokens.size()!=7){
			parsed_type = ReadSonarBase::parse_err;
			return parsed_type;
		}
		int bank,offset;
		bank = atoi(tokens[2].c_str());
		pack_n = atoi(tokens[1].c_str());

		switch(bank){
			case 1:
				offset=0;
				parsed_type=ReadSonarBase::parse_meas_b1;
				break;
			case 2:
				offset=1;
				parsed_type=ReadSonarBase::parse_meas_b2;
				break;
			case 4:
				offset=2;
				parsed_type=ReadSonarBase::parse_meas_b3;
				break;
			case 8:
				offset = 4;
				parsed_type=ReadSonarBase::parse_meas_b4;
				break;
			default:
				parsed_type=ReadSonarBase::parse_err;
				return parsed_type;
				break;
		}
		
		for (int i=0;i<4;i++){
		    if (offset != 4) {
		        /* TODO: ignoring offset = 4 to avoid having two connectors
		         * pushing data to measure[WEST]...
		         */
		        measure[i*3+offset]=to_meter*(float)atoi(tokens[i+3].c_str());
		    }
		}
	}
	return parsed_type;
}

char * ReadSonarBase::getParsedLine(){
	//call it only after parse line!!!
	return tmp_buf;
}


unsigned int ReadSonarBase::getLineToParseNum(){
	return buffer->getLineCount();
}

void ReadSonarBase::tokenize(const std::string& str,
                      std::vector<std::string>& tokens,
                      const std::string& delimiters ){
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}
