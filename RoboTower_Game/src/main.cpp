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

#include "RosComunication.h"
#include "GameControl.h"
#include "RTMainWindow.h"

Q_DECLARE_METATYPE (std::string)
 
int main(int argc, char **argv)
{
	qRegisterMetaType<std::string>("string");
	init(argc, argv, "Robotower_Game");
	RosComunication rosPublisher;
	GameControl gameControl(3,1);
	QApplication app(argc, argv);
	RTMainWindow mainWindow;

	rosPublisher.start();
	gameControl.start();

	/* when the main app is quitting make the spawned thread to quit, otherwise there's an error */
	QObject::connect(&app, SIGNAL(aboutToQuit()), &rosPublisher, SLOT(quitNow()));
	QObject::connect(&app, SIGNAL(aboutToQuit()), &gameControl, SLOT(quitNow()));
	QObject::connect(&rosPublisher, SIGNAL(rosQuits()), &app, SLOT(quit()));

	QObject::connect(&gameControl, SIGNAL(updatedTimeAndPoints(int,int)), &mainWindow, SLOT(updateData(int,int)));
	QObject::connect(&rosPublisher, SIGNAL(rfidRecieved(std::string)), &gameControl, SLOT(disableRFID(std::string)));
	QObject::connect(&gameControl, SIGNAL(updatedRfidStatus(int, bool)), &mainWindow, SLOT(updateCardStatus(int,bool)));
	QObject::connect(&rosPublisher, SIGNAL(towersUpdate(int)), &gameControl, SLOT(updateTowers(int)));
	QObject::connect(&gameControl, SIGNAL(towersUpdate(int, int)), &mainWindow, SLOT(updateTowers(int, int)));
	QObject::connect(&gameControl, SIGNAL(endGame(int,int,int)), &mainWindow, SLOT(updateHistory(int,int,int)));
	QObject::connect(&gameControl, SIGNAL(robotIsEnabled(bool)), &rosPublisher, SLOT(enableIsaac(bool)));
	QObject::connect(&gameControl, SIGNAL(rfidEnableNotification(std::string)), &rosPublisher, SLOT(enableRFID(std::string)));
	QObject::connect(&gameControl, SIGNAL(updateRemainingTime(int)), &mainWindow, SLOT(updateSetupPopup(int)));

	QObject::connect(&mainWindow, SIGNAL(start()), &gameControl, SLOT(startGame()));
	QObject::connect(&mainWindow, SIGNAL(stop()), &gameControl, SLOT(stopGame()));
	QObject::connect(&mainWindow, SIGNAL(togglePause()), &gameControl, SLOT(togglePause()));
	QObject::connect(&mainWindow, SIGNAL(newGame()), &gameControl, SLOT(resetGame()));

	mainWindow.show();

	return app.exec();
} 
