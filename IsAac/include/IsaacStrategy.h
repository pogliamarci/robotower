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
		
		RfidAction lastAction;

		int sonar[CARDINAL_POINTS];
		
		bool tower_found;
		int tower_position;
		bool factory_found;
		int factory_position;
		
		BlockedTimeCalculator sonarBuffer;
		int detectedTime;
		int randomTime;
		
		int towerDistance;
		int factoryDistance;

		int randomAhead;
		int randomSearch;
		
		int tanSpeed;
		int rotSpeed;
		
		int timer;
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
		~IsaacStrategy();
	private:
		void modifySensors(RfidAction action);
		void modifyActuators(RfidAction action);
		void useBrian();
		void updateSensors(SensorStatus& sensorStatus);
		void parseBrianResults();
		void resetVision();
		void updateRandomValues();
		void getAction(SensorStatus& sensorStatus);
};

#endif
