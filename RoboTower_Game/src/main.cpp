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

#include <QtGui>
#include <QThread>

#include "RosPublisher.h"
#include "RTMainWindow.h"
 
int main(int argc, char **argv)
{
	init(argc, argv, "Robotower_Game");
	RosPublisher rosPublisher;
	rosPublisher.start();
	QApplication app(argc, argv);

	QObject::connect(&app, SIGNAL(aboutToQuit()), &rosPublisher, SLOT(quitNow()));
	QObject::connect(&rosPublisher, SIGNAL(rosQuits()), &app, SLOT(quit()));

	RTMainWindow mainWindow;

	mainWindow.show();
	return app.exec();
} 
