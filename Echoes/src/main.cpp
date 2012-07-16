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
#include "TowerProcesser.h"
#include "SonarProcesser.h"
#include "RfidProcesser.h"
#include "ReadSonar.h"
#include "LedParser.h"
#include <iostream>

#include "ros/ros.h"
#include "Echoes/Sonar.h"
#include "Echoes/Rfid.h"
#include "Echoes/Towers.h"
#include "Echoes/Led.h"

using namespace std;

void parseItAll(ReadSonar& read_sonar, Dispatcher& d)
{
	if(read_sonar.readData()==0)
	{
		unsigned int n_line = read_sonar.getLineToParseNum();

		for(unsigned int i=0;i<n_line;i++)
		{
			d.dispatch(read_sonar.getLine());
		}
	}
}

int main(int argc, char** argv)
{
	char* serialFilename = "/dev/ttyUSB0";
	if(argc == 2) {
		serialFilename = argv[1];
	}

	ReadSonar read_sonar(serialFilename);
	Dispatcher dispatcher;


	/* Initialise ROS */
	ros::init(argc, argv, "Echoes");

	ros::NodeHandle ros_node;
	ros::ServiceServer led_service;

	LedParser lp(&read_sonar);

	/* Initialisation as publisher of sonar_data msgs */
	ros::Publisher sonar_data_pub = ros_node.advertise<Echoes::Sonar>("sonar_data", 1000);
	ros::Publisher rfid_data_pub = ros_node.advertise<Echoes::Rfid>("rfid_data", 1000);
	ros::Publisher towers_data_pub = ros_node.advertise<Echoes::Towers>("towers_data", 1000);
	led_service = ros_node.advertiseService("led_data", &LedParser::ledCallback, &lp);

	/* configuration */
	SonarProcesser spr(sonar_data_pub);
	RfidProcesser rpr(rfid_data_pub);
	TowerProcesser tpr(towers_data_pub);
	dispatcher.addProcesser(&spr, "[SONAR]");
	dispatcher.addProcesser(&rpr, "[RFID]");
	dispatcher.addProcesser(&tpr, "[TOWER]");

	/* the great loop! */
    while (ros::ok())
    {
    	parseItAll(read_sonar, dispatcher);
		ros::spinOnce();
    }
	return EXIT_SUCCESS;

}
