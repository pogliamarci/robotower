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

#include "RosComunication.h"

#include "Echoes/FixedLed.h"
#include "Echoes/ResetLed.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

RosComunication::RosComunication()
{
	hasToQuit = false;
	enableIsaacPublisher = n.advertise<std_msgs::Bool>("isaac_enable", 1000);
	rfidActionPublisher = n.advertise<std_msgs::String>("rfid_action", 1000);
	rfidCardSubscriber = n.subscribe("rfid_data", 1,
			&RosComunication::fromRfidCallback, this);
	towerSubscriber = n.subscribe("towers_data", 1,
			&RosComunication::fromTowersCallback, this);
	batterySubscriber = n.subscribe("spykee_battery", 1,
			&RosComunication::fromBatteryCallback, this);
	redLedClient = n.serviceClient<Echoes::FixedLed>("red_led");
	redResetClient = n.serviceClient<Echoes::ResetLed>("reset_led");
}

void RosComunication::run()
{
	ros::Rate rate(15);
	while (ros::ok() && !hasToQuit)
	{
		ros::spinOnce();
		rate.sleep();
	}
	if (!hasToQuit)
	{
		emit rosQuits();
	}
}

void RosComunication::quitNow()
{
	hasToQuit = true;
	resetRobot();
	enableIsaac(false);
}

void RosComunication::resetRobot()
{
	Echoes::ResetLed service;
	redResetClient.call(service);
}

void RosComunication::sendAction(std::string action)
{
	std_msgs::String message;
	message.data = action;
	rfidActionPublisher.publish(message);
}

void RosComunication::enableIsaac(bool isEnabled)
{
	std_msgs::Bool message;
	message.data = isEnabled;
	enableIsaacPublisher.publish(message);
}

void RosComunication::fromRfidCallback(const Echoes::Rfid& message)
{
	emit rfidRecieved(message.id);
}

void RosComunication::fromTowersCallback(const Echoes::Towers& message)
{
	emit towersUpdate(message.towerId);
}

void RosComunication::fromBatteryCallback(const std_msgs::Int8& message)
{
	emit batteryUpdate(message.data);
}

void RosComunication::setRedLeds(int num)
{
	Echoes::FixedLed service;
	service.request.numOn = num;
	redLedClient.call(service);
}
