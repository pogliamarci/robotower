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
#include "GameConfiguration.h"

Q_DECLARE_METATYPE (std::string)
 
int main(int argc, char **argv)
{
	QApplication app(argc, argv);
	qRegisterMetaType<std::string>("string");
	init(argc, argv, "Robotower_Game");
	GameConfiguration configuration(QCoreApplication::applicationDirPath() + "/../robotower.xml");
	RosComunication rosPublisher;
	GameControl gameControl(configuration);
	RTMainWindow mainWindow(configuration);

	/* when the main app is quitting ensure all threads quit immediately,
	 * otherwise there's an error */
	QObject::connect(&app, SIGNAL(aboutToQuit()), &rosPublisher, SLOT(quitNow()));
	QObject::connect(&app, SIGNAL(aboutToQuit()), &gameControl, SLOT(quitNow()));
	QObject::connect(&rosPublisher, SIGNAL(rosQuits()), &app, SLOT(quit()));

	/* signals from the controller to the GUI */
	QObject::connect(&gameControl, SIGNAL(timeIsUpdated(int)),
			&mainWindow, SLOT(updateTime(int)));
	QObject::connect(&gameControl, SIGNAL(pointsAreUpdated(int)),
			&mainWindow, SLOT(updatePoints(int)));
	QObject::connect(&rosPublisher, SIGNAL(rfidRecieved(std::string)),
			&gameControl, SLOT(manageRfid(std::string)));
	QObject::connect(&gameControl, SIGNAL(updatedRfidStatus(int, bool)),
			&mainWindow, SLOT(updateCardStatus(int,bool)));
	QObject::connect(&gameControl, SIGNAL(updateRemainingTime(int)),
			&mainWindow, SLOT(updateSetupPopup(int)));
	QObject::connect(&gameControl, SIGNAL(endGame(int,int,int)),
			&mainWindow, SLOT(updateHistory(int,int,int)));
	QObject::connect(&gameControl, SIGNAL(towersUpdate(int, int)),
			&mainWindow, SLOT(updateTowers(int, int)));

	/* signals between ROS and the controller */
	QObject::connect(&rosPublisher, SIGNAL(towersUpdate(int)),
			&gameControl, SLOT(updateTowers(int)));
	QObject::connect(&gameControl, SIGNAL(robotIsEnabled(bool)),
			&rosPublisher, SLOT(enableIsaac(bool)));
	QObject::connect(&gameControl, SIGNAL(rfidActionNotification(std::string)),
			&rosPublisher, SLOT(sendAction(std::string)));
	QObject::connect(&gameControl, SIGNAL(mustSetLeds(int)),
			&rosPublisher, SLOT(setRedLeds(int)));

	/* signals from the GUI (button pressed and such) */
	QObject::connect(&mainWindow, SIGNAL(start()),
			&gameControl, SLOT(startGame()));
	QObject::connect(&mainWindow, SIGNAL(stop()),
			&gameControl, SLOT(stopGame()));
	QObject::connect(&mainWindow, SIGNAL(togglePause()),
			&gameControl, SLOT(togglePause()));
	QObject::connect(&mainWindow, SIGNAL(newGame()),
			&gameControl, SLOT(resetGame()));

	rosPublisher.start();
	gameControl.start();
	mainWindow.show();

	return app.exec();
}
