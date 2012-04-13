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

#include <iostream>
#include <string>
#include "brian.h"
#include "ros/ros.h"

#include "sonarbuffer.h"
#include "sensorstatus.h"
#include "sender.h"

#define FUZZYASSOC (char *) "../config/ctof.txt"
#define FUZZYSHAPES (char *) "../config/shape_ctof.txt"
#define PRIES (char *) "../config/Predicate.ini"
#define PRIESACTIONS (char *) "../config/PredicateActions.ini"
#define CANDOES (char *) "../config/Cando.ini"
#define BEHAVIORS (char *) "../config/behaviour.txt"
#define WANTERS (char *) "../config/want.txt"
#define DEFUZZYASSOC (char *) "../config/s_ftoc.txt"
#define DEFUZZYSHAPES (char *) "../config/s_shape.txt"

#define LOOPRATE 20

void sendBrianOutputs(command_list* cl, Sender& ms, ros::ServiceClient client) 
{
	if (cl == NULL || cl->empty()) 
	{
		cerr << "Ricevuta lista vuota da Brian" << endl;
		return;
	}

	int tan_speed = 0;
	int rot_speed = 0;

	Echoes::Led led_service;

	for (command_list::iterator it = cl->begin(); it != cl->end(); it++)
	{
		std::string temp = it->first;

		if (temp.compare("TanSpeed") == 0)
		{
			cout << "Ricevuta tan speed" << endl;
			tan_speed = it->second->get_set_point();
		}
		else if (temp.compare("RotSpeed") == 0)
		{
			cout << "Ricevuta rot speed" << endl;
			rot_speed = it->second->get_set_point();
		} 
		else if (temp.compare("GreenLed") == 0) 
		{
			led_service.request.editGreen = true;
			led_service.request.greenIsOn = it->second->get_set_point();
			client.call(led_service);
		} 
		else 
		{
			cout << "Ricevuto qualcos'altro" << endl;
		}
	}

	cout << "rot_speed: " << rot_speed << " tan_speed: " << tan_speed << endl;

	/* send commands to actuators */
	ms.sendMotionMessage(rot_speed, tan_speed);
}

int main(int argc, char** argv)
{
	//random variables and functions
	int random_timer   = 0;
	int random_search  = 0;
	int detected_timer = 0;
	int random_ahead   = 0;
	srand((unsigned)time(NULL));
	
	//sonar variable
	SonarBuffer sonarBuffer;

	//data
	SensorStatus sensors;
	crisp_data_list* cdl;
	
	
	/* reliability (not used ==> set to 1) */
	const int reliability = 1;

	//ros initialization
	ros::init(argc, argv, "isaac");
	ros::NodeHandle ros_node = ros::NodeHandle();
	ros::Subscriber sonar_sub = ros_node.subscribe("sonar_data", 1,
			&SensorStatus::fromSonarCallback, &sensors);
	ros::Subscriber vision_sub = ros_node.subscribe("vision_results", 1,
				&SensorStatus::fromVisionCallback, &sensors);
	ros::ServiceClient client = ros_node.serviceClient<Echoes::Led>("led_data");

	ros::Rate loop_rate(LOOPRATE);

	Sender message_sender(ros_node);

	MrBrian brian = MrBrian(FUZZYASSOC, FUZZYSHAPES,
				PRIES, PRIESACTIONS,
				CANDOES, BEHAVIORS,
				WANTERS, DEFUZZYASSOC,
				DEFUZZYSHAPES);

	while (ros::ok())
	{
		/* let's get (a pointer to) the input data list and clear it */
		cdl = (brian.getFuzzy())->get_crisp_data_list();
		cdl->clear();

		cout << "sonar: N " << sensors.getSonar(NORTH) << " , " <<
				sensors.getSonar(SOUTH) << " , " << sensors.getSonar(EAST)
				<< " , " << sensors.getSonar(WEST) << endl;
		int sonar_north = sensors.getSonar(NORTH);
		sonarBuffer.insert(sonar_north);
		sonarBuffer.setTempoBloccato();

		/* update inputs */
		cdl->add(new crisp_data("DistanceNorth",sonar_north , reliability));
		cdl->add(new crisp_data("DistanceSouth", sensors.getSonar(SOUTH), reliability));
		cdl->add(new crisp_data("DistanceEast", sensors.getSonar(EAST), reliability));
		cdl->add(new crisp_data("DistanceWest", sensors.getSonar(WEST), reliability));
		cdl->add(new crisp_data("InvisibleObstacle", sonarBuffer.getTempoBloccato(), reliability));
		if (sensors.isTowerDetected())
		{
			cout << "tower detected: pos = " << sensors.getTowerPosition() << endl;
			detected_timer=0;
		}
		else detected_timer++;

		cdl->add(new crisp_data("TowerDetected", sensors.isTowerDetected(), reliability));
		cdl->add(new crisp_data("TowerPosition", sensors.getTowerPosition(), reliability));
		cdl->add(new crisp_data("FactoryDetected", sensors.isFactoryDetected(), reliability));
		cdl->add(new crisp_data("FactoryPosition", sensors.getFactoryPosition(), reliability));

		/* random data set to ros */
		if(detected_timer == LOOPRATE)
		{
			random_ahead = rand() % 100;
			detected_timer = 0;
		}
		
		if((random_timer++) == 4*LOOPRATE)
		{
			random_search = rand() % 100;
			random_timer = 0;
		}
		cout << "random src: " << random_search << endl;
		cdl->add(new crisp_data("RandomSearch", random_search, reliability));
		cdl->add(new crisp_data("RandomAhead", random_ahead, reliability));
		
		/* let's start it all */
		brian.run();
		brian.debug();

		/* parse outputs from brian and send them to actuators */
		command_list* cl = brian.getFuzzy()->get_command_singleton_list();
		sendBrianOutputs(cl, message_sender, client);

		message_sender.sendDebugMessage(sonarBuffer.calcolaMedia(),
				sonarBuffer.calcolaStdDev(), sonarBuffer.getTempoBloccato());

		cl->clear();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
