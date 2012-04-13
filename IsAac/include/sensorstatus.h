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

#define CARDINAL_POINTS 4

#include "Echoes/Sonar.h"
#include "Echoes/Led.h"
#include "Vision/Results.h"

typedef enum
{
	NORTH, SOUTH, EAST, WEST
} CardinalPoint;

class SensorStatus 
{
	private:
		int sonar[CARDINAL_POINTS];
		bool tower_found;
		int tower_position;
		bool factory_found;
		int factory_position;
	public:
		SensorStatus();
		void fromSonarCallback(const Echoes::Sonar& message);
		void fromVisionCallback(const Vision::Results& message);
		/* some getters (declared here as inline) */
		inline bool isTowerDetected() 
		{
			return tower_found;
		}
		inline int getTowerPosition() 
		{
			return tower_position;
		}
		inline bool isFactoryDetected() 
		{
			return factory_found;
		}
		inline int getFactoryPosition() 
		{
			return factory_position;
		}
		inline int getSonar(CardinalPoint p)
		{
			return sonar[p];
		}
};
