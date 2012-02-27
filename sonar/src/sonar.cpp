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

#include "ReadSonar.h"
#include <iostream>
#include "ros/ros.h"
#include "sonar/Sonar.h"
#include "sonar/Led.h"
#include "LedParser.h"

#define SERIAL_DEVICE_FILENAME "/dev/ttyUSB0"

#define NORTH 			0
#define SOUTH			1
#define NORTH_EAST		2

#define EAST			6
#define WEST			7
#define NORTH_WEST		8

using namespace std;

ReadSonar* initialize();
void loop(ReadSonar*, ros::Publisher*);
void readDataFromSonar(ReadSonar*);
/*void ledCallback(Led);*/
/*
void ledCallback(Led led_data) {



} */


int main(int argc, char** argv) 
{
	try 
	{
		ReadSonar* readSonar = initialize();
		LedParser led(readSonar);

		ros::init(argc, argv, "sonar");
		ros::NodeHandle n;
		
		/* initialization as publisher of sonar_data msgs */
		ros::Publisher sonar_data_pub = n.advertise<sonar::Sonar>("sonar_data", 1000);
		/* ros::ServiceServer advertiseService("led_data", ledCallback); */
		loop(readSonar, &sonar_data_pub);
		delete readSonar;
		return 0;
	} 
	catch  ( ReadSonarDeviceException &e )
	{
		cerr << "Read sonar device exception\n";
		cerr << e.what() << "\n";
	}
	return 1;
}

ReadSonar* initialize() 
{
	ReadSonar* readSonar = new ReadSonar( SERIAL_DEVICE_FILENAME, 1 );
	if ( readSonar ) 
	{
		readSonar->sendRun();
		char c = (char) 0;
		readSonar->sendStringCommand(&c,1);
		sleep(3);
	}
	return readSonar;
}

void loop(ReadSonar* readSonar, ros::Publisher* sonar_data_pub_ptr) 
{
	ros::Publisher sonar_data_pub = *sonar_data_pub_ptr;
	while(ros::ok())
	{
		readDataFromSonar(readSonar);
		/* build the message and publish it */
		sonar::Sonar* msg = new sonar::Sonar();
		msg->north = readSonar->getMeasure(NORTH);
		msg->south = readSonar->getMeasure(SOUTH);
		msg->east = readSonar->getMeasure(EAST);
		msg->west = readSonar->getMeasure(WEST);
		sonar_data_pub.publish(*msg);
		ros::spinOnce();
		delete msg;
	}
}

void readDataFromSonar(ReadSonar* readSonar) 
{
	static int meas_progress=0;
	static int line_readed=0;
	if(readSonar->readData()==0)
	{
		unsigned int n_line;
		n_line=readSonar->getLineToParseNum();
		line_readed+=n_line;

		for(unsigned int i=0;i<n_line;i++)
		{
			switch(readSonar->parseLine())
			{
				case ReadSonar::parse_err:
					cerr << "Error in parse method :" << readSonar->getParsedLine() << endl;
					cout << "NORD: " << readSonar->getMeasure(NORTH) << endl;
					cout << "SUD: "  << readSonar->getMeasure(SOUTH) << endl;
					meas_progress=0;
					break;
				case ReadSonar::parse_meas_b1:
					//FileLogger::write("found meas group 1:");
					//FileLogger::writeLine(readSonar->getParsedLine());
					if(meas_progress==0)meas_progress++;
					break;
				case ReadSonar::parse_meas_b2:
					//FileLogger::write("found meas group 2:");
					//FileLogger::writeLine(readSonar->getParsedLine());
					if(meas_progress==1)meas_progress++;
					break;
				case ReadSonar::parse_meas_b3:
					//FileLogger::write("found meas group 3:");
					//FileLogger::writeLine(readSonar->getParsedLine());
					if(meas_progress==2)meas_progress++;
					break;
				case ReadSonar::parse_meas_b4:
					// FileLogger::write("found meas group 4:");
					// FileLogger::writeLine(readSonar->getParsedLine());
					if(meas_progress==3)meas_progress++;
					break;
				case ReadSonar::parse_dbg:
					printf("found DBG line <%s>",readSonar->getParsedLine());
					meas_progress=0;
					break;
					//                  case ReadSonar::parse_ok:
					//                      printf("found OK line <%s>",readSonar->getParsedLine());
					//                      meas_progress=0;
					//                      break;
				case ReadSonar::parse_response_err:
					printf("found ERR line <%s>",readSonar->getParsedLine());
					meas_progress=0;
					break;
				default:
					printf("defensive programming... unrechable... mumble mumble\n");
					meas_progress=0;
					break;
			}
		}
	}
}
