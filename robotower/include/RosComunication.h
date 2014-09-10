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

#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include "ros/ros.h"
#include "echoes/Rfid.h"
#include "echoes/Towers.h"
#include "std_msgs/Int8.h"

#include <QThread>
#include <QObject>

#include <string>

using namespace ros;
 
class RosComunication : public QThread
{
Q_OBJECT

private:
	NodeHandle n;
	Publisher enableIsaacPublisher;
	Publisher rfidActionPublisher;
	Subscriber rfidCardSubscriber;
	Subscriber towerSubscriber;
	Subscriber batterySubscriber;
	ServiceClient redLedClient;
	ServiceClient redResetClient;
	bool hasToQuit;
public:
	RosComunication();
	void run();
private:
	void fromRfidCallback(const echoes::Rfid& message);
	void fromTowersCallback(const echoes::Towers& message);
	void fromBatteryCallback(const std_msgs::Int8& message);
public slots:
	void resetRobot();
	void sendAction(std::string id);
	void enableIsaac(bool isEnabled);
	void setRedLeds(int num);
	void quitNow(); // stops the thread when the application is quitting, reset the robot and stop IsAac
signals:
	void rosQuits(); // triggered if ros::ok() is not true anymore
	void rfidRecieved(std::string id); //triggered when arrives a RFID tag
	void towersUpdate(int towerNumber);
	void batteryUpdate(int data);
};

#endif
