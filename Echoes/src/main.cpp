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
	Dispatcher dispatcher;
	ReadSonar read_sonar(SERIAL_DEVICE_FILENAME, 1);

	/* Initialise ROS */
	ros::init(argc, argv, "Echoes");

	ros::NodeHandle ros_node;
	ros::ServiceServer led_service;

	/* a little bit of tests for the parser... */
	dispatcher.dispatch("Questa e' una prova");
	dispatcher.dispatch("Questa e' una altra prova");
	dispatcher.dispatch("[RFID] Questo e' uno RFID");
	dispatcher.dispatch("[SONAR] Questi sono dei sonar, finalmente");
	dispatcher.dispatch("[SONAR] N:0,S:5,E:8,W:89");
	dispatcher.dispatch("[RFID] 0F03182595A4");

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
