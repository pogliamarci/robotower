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

	lastAction = "";

	for (int i = 0; i < CARDINAL_POINTS; i++)
		sonar[i] = 0;

	detectedTime = 0;
	randomTime = 0;

	randomAhead = 0;
	randomSearch = 0;

	towerDistance = 0;
	factoryDistance = 0;

	tanSpeed = 0;
	rotSpeed = 0;

	timer = 0;

	resetVision();
}

void IsaacStrategy::activateStrategy(SensorStatus& sensorStatus)
{
	getAction(sensorStatus);
	updateSensors(sensorStatus);
	modifySensors();
	useBrian();
	parseBrianResults();
	modifyActuators();
}

void IsaacStrategy::getAction(SensorStatus& sensorStatus)
{
	if (timer != 0)
		return;
	
	if(sensorStatus.hasValidAction())
	{
		lastAction = sensorStatus.consumeLastAction();
		action_rand_direction = rand() % 2;
		timer = 5 * LOOPRATE;
	} else lastAction = "";
}

void IsaacStrategy::resetVision()
{
	tower_found = false;
	tower_position = 0;
	factory_found = false;
	factory_position = 0;
}

void IsaacStrategy::modifySensors()
{
	if(lastAction == "disable_vision")
	{
		resetVision();
	}
}

void IsaacStrategy::modifyActuators()
{
	if(lastAction == "lock_all")
	{
		tanSpeed = 0;
		rotSpeed = 0;
	} 
	else if(lastAction == "force_rotate") 
	{
		int right_track = tanSpeed - rotSpeed;
		int left_track = tanSpeed + rotSpeed;
		if(action_rand_direction == 1)
			left_track = 0;
		else right_track = 0;
		tanSpeed = (right_track + left_track) / 2;
		rotSpeed = (left_track - right_track) / 2;
	}
	else if(lastAction == "go_back" && canGoBack)
	{
		if(tanSpeed > 0) tanSpeed = -tanSpeed;
		rotSpeed = 0;
	}
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
	cdl->add(
			new crisp_data("InvisibleObstacle", sonarBuffer.getTempoBloccato(),
					reliability));
	//sensor (difference betweeen north sonar distance and vision distance)
	int difference = sonar[NORTH] -	(tower_found)? (towerDistance) : (factoryDistance);
	cdl->add(
			new crisp_data("SensorMatches", difference , reliability));
	//vision
	cdl->add(new crisp_data("TowerDetected", tower_found, reliability));
	cdl->add(new crisp_data("TowerPosition", tower_position, reliability));
	cdl->add(new crisp_data("FactoryDetected", factory_found, reliability));
	cdl->add(new crisp_data("FactoryPosition", factory_position, reliability));
	//random
	cdl->add(new crisp_data("RandomSearch", randomSearch, reliability));
	cdl->add(new crisp_data("RandomAhead", randomAhead, reliability));

	brian->run();
	brian->debug(); // print on stdout debug information

	/* deallocate what has been allocated!,
	 * this is quite ugly but _should_ avoid some mem leaks...
	 * The list itself will be cleared out at the _next_ iteration ;)*/
	crisp_data_list::iterator itr;
	for(itr = cdl->begin(); itr != cdl->end(); itr++){
		delete (*itr).second;
	}
}

void IsaacStrategy::updateSensors(SensorStatus& sensorStatus)
{
	for (int i = 0; i < CARDINAL_POINTS; i++)
	{
		sonar[i] = sensorStatus.getSonar((CardinalPoint) i);
	}
	tower_found = sensorStatus.isTowerDetected();
	tower_position = sensorStatus.getTowerPosition();
	factory_found = sensorStatus.isFactoryDetected();
	factory_position = sensorStatus.getFactoryPosition();
	towerDistance = sensorStatus.getTowerDistance();
	factoryDistance = sensorStatus.getFactoryDistance();

	updateRandomValues();

	if (timer > 0)
		timer--;
	sonarBuffer.insert(sensorStatus.getSonar(NORTH));
}

void IsaacStrategy::updateRandomValues()
{
	if (tower_found)
	{
		detectedTime = 0;
	}
	else detectedTime++;

	if (detectedTime == LOOPRATE)
	{
		randomAhead = rand() % 100;
		detectedTime = 0;
	}

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
		else if(temp.compare("CanGoBack") == 0)
		{
			canGoBack = it->second->get_set_point();
		}
	}

	cl->clear();
}

IsaacStrategy::~IsaacStrategy()
{
	delete brian;
}
