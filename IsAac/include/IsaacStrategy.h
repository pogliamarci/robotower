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

#ifndef ISAACSTRATEGY_H
#define ISAACSTRATEGY_H

#include "brian.h"
#include "SensorStatus.h"
#include "BlockedTimeCalculator.h"
#include "IsaacMemory.h"

#define FUZZYASSOC (char *) "../config/ctof.txt"
#define FUZZYSHAPES (char *) "../config/shape_ctof.txt"
#define PRIES (char *) "../config/Predicate.ini"
#define PRIESACTIONS (char *) "../config/PredicateActions.ini"
#define CANDOES (char *) "../config/Cando.ini"
#define BEHAVIORS (char *) "../config/behaviour.txt"
#define WANTERS (char *) "../config/Want.ini"
#define DEFUZZYASSOC (char *) "../config/s_ftoc.txt"
#define DEFUZZYSHAPES (char *) "../config/s_shape.txt"

const int LOOPRATE = 15;

class IsaacStrategy
{
private:
	MrBrian* brian;

	int sonar[CARDINAL_POINTS];

	bool tower_found;
	int tower_position;
	bool factory_found;
	int factory_position;

	BlockedTimeCalculator sonarBuffer;
	IsaacMemory memory;
	int detectedTime;

	int towerDistance;
	int factoryDistance;

	int randomAhead;
	int randomSearch;

	int tanSpeed;
	int rotSpeed;
	bool canGoBack;

	/* variables to manage the action triggered by RFIDs */
	std::string lastAction;
	int timer;
	char action_rand_direction;

public:
	IsaacStrategy();
	void activateStrategy(SensorStatus& sensorStatus);
	inline int getRotSpeed()
	{
		return rotSpeed;
	}
	inline int getTanSpeed()
	{
		return tanSpeed;
	}
	inline bool isTrapped()
	{
		return lastAction != "";
	}
	inline bool hasSeenSomething()
	{
		return tower_found || factory_found;
	}
	~IsaacStrategy();
private:
	void modifySensors();
	void modifyActuators();
	void useBrian();
	void updateSensors(SensorStatus& sensorStatus);
	void parseBrianResults();
	void resetVision();
	void updateRandomValues();
	void getAction(SensorStatus& sensorStatus);
};

#endif
