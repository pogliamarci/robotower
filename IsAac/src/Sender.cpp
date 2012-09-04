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

#include "Echoes/BlinkingLed.h"
#include "SpyKee/Motion.h"

Sender::Sender(ros::NodeHandle& n) : yellowBlinking(false), greenBlinks(false), ledEnabled(false)
{
	motion = n.advertise<SpyKee::Motion>("spykee_motion", 1000);
	yellowled = n.serviceClient<Echoes::BlinkingLed>("yellow_led");
	greenled = n.serviceClient<Echoes::BlinkingLed>("green_led");
}

void Sender::sendMotionMessage(int tanSpeed, int rotSpeed)
{
	SpyKee::Motion msg;
	msg.rotSpeed = rotSpeed;
	msg.tanSpeed = tanSpeed;
	this->motion.publish(msg);
}

void Sender::disableLed()
{
	if(!ledEnabled) return;
	ledEnabled = false;
	yellowBlinking = false;
	greenBlinks = false;
	sendYellow();
	sendGreen();
}

void Sender::sendYellow()
{
	Echoes::BlinkingLed service;
	service.request.on = ledEnabled;
	service.request.blinks = yellowBlinking;
	yellowled.call(service);
}

void Sender::sendGreen()
{
	Echoes::BlinkingLed service;
	service.request.on = greenBlinks;
	service.request.blinks = greenBlinks;
	greenled.call(service);
}

void Sender::setLed(bool isTrapped, bool seenSomething)
{
	if (isTrapped && (yellowBlinking || !ledEnabled))
	{
		yellowBlinking = false;
		ledEnabled = true;
		sendYellow();
	} else if(!isTrapped && !yellowBlinking) {
		yellowBlinking = true;
		ledEnabled = true;
		sendYellow();
	}

	if (seenSomething && !greenBlinks)
	{
		greenBlinks = true;
		sendGreen();

	}
	else if (!seenSomething && greenBlinks)
	{
		greenBlinks = false;
		sendGreen();
	}
}
