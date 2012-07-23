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

#include "Sender.h"

Sender::Sender(ros::NodeHandle& n) 
{
	motion = n.advertise<SpyKee::Motion>("spykee_motion", 1000);
	debug_mediavarianza = n.advertise<IsAac::MediaVarianza>("debug_mediavarianza", 1000);
}

void Sender::sendMotionMessage(int tanSpeed, int rotSpeed)
{
	SpyKee::Motion msg;
	msg.rotSpeed = rotSpeed;
	msg.tanSpeed = tanSpeed;
	this->motion.publish(msg);
}

void Sender::sendDebugMessage(float avg, float var, int time)
{
	IsAac::MediaVarianza msg;
	msg.Average = avg;
	msg.Variance = var;
	msg.Time = time;
	this->debug_mediavarianza.publish(msg);
}
