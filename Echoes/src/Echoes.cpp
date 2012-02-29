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
#include "Echoes/Sonar.h"
#include "Echoes/Led.h"
#include "LedParser.h"

#define SERIAL_DEVICE_FILENAME "/dev/ttyUSB0"

#define NORTH 			0
#define SOUTH			1
#define NORTH_EAST		2

#define EAST			6
#define WEST			7
#define NORTH_WEST		8

using namespace std;

class EchoesManager
{
	private:
		ReadSonar* read_sonar;
		LedParser* led;
		ros::NodeHandle ros_node;
		ros::Publisher sonar_data_pub;
		ros::ServiceServer led_service;
	public:
		EchoesManager();
		~EchoesManager();
		bool ledCallback(Echoes::Led::Request&, Echoes::Led::Response&);
		void publishSonarData();
		void readSonarData();
		void sendMessage(float, float, float, float);
};

EchoesManager::EchoesManager()
{
	read_sonar = new ReadSonar( SERIAL_DEVICE_FILENAME, 1 );
	if ( read_sonar ) 
	{
		read_sonar->sendRun();
		char c = (char) 0;
		read_sonar->sendStringCommand(&c,1);
		sleep(3);
	}

	led = new LedParser(read_sonar);
	/* Initialisation as publisher of sonar_data msgs */
	sonar_data_pub = ros_node.advertise<Echoes::Sonar>("sonar_data", 1000);
	led_service = ros_node.advertiseService("led_data", &EchoesManager::ledCallback, this);
}

EchoesManager::~EchoesManager()
{
	delete led;
	delete read_sonar;
}

bool EchoesManager::ledCallback(Echoes::Led::Request& request, Echoes::Led::Response& response) 
{
	bool yellow_on[4];
	if (request.editGreen == true) 
	{
		led->Green(request.greenIsOn);
	}
	if (request.editRed == true) 
	{
		led->Red(request.redNumOn);
	}
	if (request.editYellow == true) 
	{
		for (int i = 0; i < 4; i++)
		{
			if (request.yellowIsOn[i] == 1) yellow_on[i] = true;
			else yellow_on[i] = false;
		}
		led->Yellow(yellow_on);
	}
	led->SendToLed();
	response.requestSuccessful = true;
	return true;
}

void EchoesManager::publishSonarData() 
{
	this->readSonarData();
	this->sendMessage(read_sonar->getMeasure(NORTH),
			  read_sonar->getMeasure(SOUTH),
			  read_sonar->getMeasure(EAST),
			  read_sonar->getMeasure(WEST));
}

void EchoesManager::readSonarData() 
{
	static int meas_progress=0;
	static int line_readed=0;
	if(read_sonar->readData()==0)
	{
		unsigned int n_line;
		n_line=read_sonar->getLineToParseNum();
		line_readed+=n_line;

		for(unsigned int i=0;i<n_line;i++)
		{
			switch(read_sonar->parseLine())
			{
				case ReadSonar::parse_err:
					cerr << "Error in parse method :" << read_sonar->getParsedLine() << endl;
					cout << "NORD: " << read_sonar->getMeasure(NORTH) << endl;
					cout << "SUD: "  << read_sonar->getMeasure(SOUTH) << endl;
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
					printf("found DBG line <%s>",read_sonar->getParsedLine());
					meas_progress=0;
					break;
					//                  case ReadSonar::parse_ok:
					//                      printf("found OK line <%s>",readSonar->getParsedLine());
					//                      meas_progress=0;
					//                      break;
				case ReadSonar::parse_response_err:
					printf("found ERR line <%s>",read_sonar->getParsedLine());
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

void EchoesManager::sendMessage(float north, float south, float east, float west) 
{
	Echoes::Sonar* msg = new Echoes::Sonar();
	msg->north = north;
	msg->south = south;
	msg->east = east;
	msg->west = west;
	sonar_data_pub.publish(*msg);
	delete msg;
}

int main(int argc, char** argv) 
{
    /* Initialise ROS */
    ros::init(argc, argv, "Echoes");
    try
	{
		EchoesManager manager;
		while (ros::ok()) 
		{
			manager.publishSonarData();
			ros::spinOnce();
		}
		return EXIT_SUCCESS;
	} 
	catch  ( ReadSonarDeviceException &e )
	{
		cerr << "Read sonar device exception\n";
		cerr << e.what() << "\n";
	}
	return EXIT_FAILURE;
}
