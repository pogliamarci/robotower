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

#include "IsaacStrategy.h"

IsaacStrategy::IsaacStrategy()
{
	brian = new MrBrian(FUZZYASSOC, FUZZYSHAPES, PRIES, PRIESACTIONS, CANDOES,
			BEHAVIORS, WANTERS, DEFUZZYASSOC, DEFUZZYSHAPES);

	lastAction = nothing;

	for (int i = 0; i < CARDINAL_POINTS; i++)
		sonar[i] = 0;

	detectedTime = 0;
	randomTime = 0;

	randomAhead = 0;
	randomSearch = 0;

	tanSpeed = 0;
	rotSpeed = 0;

	timer = 0;

	resetVision();
}

void IsaacStrategy::activateStrategy(SensorStatus& sensorStatus)
{
	getAction(sensorStatus);
	updateSensors(sensorStatus);
	modifySensors(lastAction);
	useBrian();
	parseBrianResults();
	modifyActuators(lastAction);
}

void IsaacStrategy::getAction(SensorStatus& sensorStatus)
{
	if(timer == 0)
	{
		lastAction = sensorStatus.consumeLastAction();
		if(lastAction != nothing) {
			switch(lastAction) {
			case lock_all:
				cerr << "Lock all" << endl;
				break;
			case disable_vision:
				cerr << "Disable vision" << endl;
				break;
			case force_rotate_left:
			case force_rotate_right:
				cerr << "Force rotate" << endl;
				break;
			case nothing:
				cerr << "NOTHING ERROR ERROR ERROR" << endl;
				break;
			}
			timer = 5 * LOOPRATE;
		}
	}
}

void IsaacStrategy::resetVision()
{
	tower_found = false;
	tower_position = 0;
	factory_found = false;
	factory_position = 0;
}

void IsaacStrategy::modifySensors(RfidAction action)
{
	switch (action)
	{
	case disable_vision:
		resetVision();
		break;
	default:
		break;
	}
}

void IsaacStrategy::modifyActuators(RfidAction action)
{
	int right_track = tanSpeed - rotSpeed;
	int left_track = tanSpeed + rotSpeed;

	switch (action)
	{
	case lock_all:
		right_track = 0;
		left_track = 0;
		break;
	case force_rotate_left:
		left_track = 0;
		break;
	case force_rotate_right:
		right_track = 0;
		break;
	default:
		break;
	}

	tanSpeed = (right_track + left_track) / 2;
	rotSpeed = (left_track - right_track) / 2;

}

void IsaacStrategy::useBrian()
{
	/* reliability (not used ==> set to 1) */
	const int reliability = 1;

	/* let's get (a pointer to) the input data list and clear it */
	crisp_data_list* cdl = (brian->getFuzzy())->get_crisp_data_list();
	cdl->clear();

	/* update inputs */
	//sonar
	cdl->add(new crisp_data("DistanceNorth", sonar[NORTH], reliability));
	cdl->add(new crisp_data("DistanceSouth", sonar[SOUTH], reliability));
	cdl->add(new crisp_data("DistanceEast", sonar[EAST], reliability));
	cdl->add(new crisp_data("DistanceWest", sonar[WEST], reliability));
	cdl->add(new crisp_data("InvisibleObstacle", sonarBuffer.getTempoBloccato(), reliability));
	//sensor (difference betweeen north sonar distance and vision distance)
	//FIXME errore! non è vero che fa così
	cdl->add(
			new crisp_data("SensorMatches", sonar[NORTH] - sonar[NORTH],
					reliability));
	//vision
	cdl->add(new crisp_data("TowerDetected", tower_found, reliability));
	cdl->add(new crisp_data("TowerPosition", tower_position, reliability));
	cdl->add(new crisp_data("FactoryDetected", factory_found, reliability));
	cdl->add(new crisp_data("FactoryPosition", factory_position, reliability));
	//random
	cdl->add(new crisp_data("RandomSearch", randomSearch, reliability));
	cdl->add(new crisp_data("RandomAhead", randomAhead, reliability));

	brian->run();
	brian->debug();

}

void IsaacStrategy::updateSensors(SensorStatus& sensorStatus)
{
	for (int i = 0; i < CARDINAL_POINTS; i++)
		sonar[i] = sensorStatus.getSonar((CardinalPoint) i);
	tower_found = sensorStatus.isTowerDetected();
	tower_position = sensorStatus.getTowerPosition();
	factory_found = sensorStatus.isFactoryDetected();
	factory_position = sensorStatus.getFactoryPosition();
	updateRandomValues();

	if (timer > 0)
		timer--;
	sonarBuffer.insert(sensorStatus.getSonar(NORTH));
}

void IsaacStrategy::updateRandomValues()
{
	//Updates detectedTime
	if (tower_found)
	{
		cout << "tower detected: pos = " << tower_position << endl;
		detectedTime = 0;
	}
	else
		detectedTime++;

	//updates Random Ahead
	if (detectedTime == LOOPRATE)
	{
		randomAhead = rand() % 100;
		detectedTime = 0;
	}

	//updates random Search
	if ((randomTime++) == 4 * LOOPRATE)
	{
		randomSearch = rand() % 100;
		randomTime = 0;
	}
}

void IsaacStrategy::parseBrianResults()
{
	tanSpeed = 0;
	rotSpeed = 0;

	/* parse outputs from brian and send them to actuators */
	command_list* cl = brian->getFuzzy()->get_command_singleton_list();

	if (cl == NULL || cl->empty())
	{
		cerr << "Ricevuta lista vuota da Brian" << endl;
		return;
	}

	for (command_list::iterator it = cl->begin(); it != cl->end(); it++)
	{
		std::string temp = it->first;

		if (temp.compare("TanSpeed") == 0)
		{
			tanSpeed = it->second->get_set_point();
		}
		else if (temp.compare("RotSpeed") == 0)
		{
			rotSpeed = it->second->get_set_point();
		}
		else if (temp.compare("GreenLed") == 0)
		{
			cout << "Green led posto a " << it->second->get_set_point() << endl;
			/*
			 Echoes::Led led_service;
			 led_service.request.editGreen = true;
			 led_service.request.greenIsOn = it->second->get_set_point();
			 client.call(led_service);
			 */
		}
	}

	cl->clear();
}

IsaacStrategy::~IsaacStrategy()
{
	delete brian;
}
