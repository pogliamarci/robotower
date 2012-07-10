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

#include "Dispatcher.h"
#include "SonarProcesser.h"
#include "RfidProcesser.h"
#include "ReadSonar.h"
#include "LedParser.h"
#include <iostream>

#include "ros/ros.h"
#include "Echoes/Sonar.h"
#include "Echoes/Led.h"

#define SERIAL_DEVICE_FILENAME "/dev/ttyUSB0"

using namespace std;

void parseItAll(ReadSonar& read_sonar, Dispatcher& d)
{
	static int line_readed=0;
	if(read_sonar.readData()==0)
	{
		unsigned int n_line = read_sonar.getLineToParseNum();
		line_readed += n_line;

		for(unsigned int i=0;i<n_line;i++)
		{
			d.dispatch(read_sonar.getLine());
		}
	}
}

int main(int argc, char** argv)
{
	char* sdfn = SERIAL_DEVICE_FILENAME;
	if(argc == 2) {
		sdfn = argv[1];
	}

	ReadSonar read_sonar(sdfn);
	Dispatcher dispatcher;


	/* Initialise ROS */
	ros::init(argc, argv, "Echoes");

	ros::NodeHandle ros_node;
	ros::ServiceServer led_service;

	LedParser lp(&read_sonar);

	/* Initialisation as publisher of sonar_data msgs */
	ros::Publisher sonar_data_pub = ros_node.advertise<Echoes::Sonar>("sonar_data", 1000);
	led_service = ros_node.advertiseService("led_data", &LedParser::ledCallback, &lp);

	/* configuration */
	SonarProcesser spr(sonar_data_pub);
	RfidProcesser rpr;
	dispatcher.addProcesser(&spr, "[SONAR]");
	dispatcher.addProcesser(&rpr, "[RFID]");

	/* the great loop! */
    while (ros::ok())
    {
    	parseItAll(read_sonar, dispatcher);
		ros::spinOnce();
    }
	return EXIT_SUCCESS;

}
