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

#include "RTMainWindow.h"
#include "RTPopupTimer.h"
#include "RTCurrentGameWidget.h"
#include "RTCards.h"

#include <iostream>

void RTMainWindow::setupToolbar()
{
	QToolBar* fileToolBar = addToolBar(tr("MainBar"));
	newGameAction = new QAction(tr("&New game"), this);
	fileToolBar->addAction(newGameAction);
	addToolBar(Qt::TopToolBarArea, fileToolBar);
}

QLayout* RTMainWindow::setupButtons()
{
	QVBoxLayout* btnLayout = new QVBoxLayout();
	startBtn = new QPushButton("&Start");
	stopBtn = new QPushButton("St&op");
	btnLayout->addWidget(startBtn);
	btnLayout->addWidget(stopBtn);
	stopBtn->setEnabled(false);
	return btnLayout;
}

QWidget* RTMainWindow::setupStats()
{
	QGroupBox* statsGroupBox = new QGroupBox();
	QGridLayout* statsLayout = new QGridLayout();
	statWon = new QLabel("0");
	statTotalScore = new QLabel("0");
	statLost = new QLabel("0");
	statsGroupBox->setTitle("Stats");
	statsGroupBox->setLayout(statsLayout);

	statsLayout->addWidget(new QLabel("Won matches: "), 1, 1);
	statsLayout->addWidget(statWon, 1, 2);
	statsLayout->addWidget(new QLabel("Lost matches: "), 2, 1);
	statsLayout->addWidget(statLost, 2, 2);
	statsLayout->addWidget(new QLabel("Total score: "), 3, 1);
	statsLayout->addWidget(statTotalScore, 3, 2);

	return statsGroupBox;
}

void RTMainWindow::setupLayout(GameConfiguration& config)
{
	QWidget* mainWidget = new QWidget();
	QHBoxLayout* mainLayout = new QHBoxLayout();
	QVBoxLayout* leftLayout = new QVBoxLayout();
	setCentralWidget(mainWidget);

	/* Children widgets */
	currentGame = new RTCurrentGameWidget();
	cardsLayout = new RTCards(config);

	mainWidget->setLayout(mainLayout);
	mainLayout->addStretch(1);
	mainLayout->addLayout(leftLayout, 12);
	mainLayout->addLayout(cardsLayout, 20);

	batteryStatus = new QProgressBar();
	batteryStatus->setObjectName(QString::fromUtf8("BatStat"));
	batteryStatus->setMinimum(0);
	batteryStatus->setMaximum(100);
	batteryStatus->setValue(0);

	const char * batStyleSheet =
			"QProgressBar {"
			"	border: 1px solid black;"
			"	border-radius: 4px;"
			"	color: black;"
			"	text-align: center;"
			"	font-weight: bold;"
			"}"
			"QProgressBar::chunk#BatStat {"
			"	background: qlineargradient(x1: 0, y1: 0.5, x2: 1.5, y2: 0.5, stop: 0 red, stop: 1 white);"
			"}"
			"QProgressBar::chunk#BatStat[batterystatus=\"charging\"] {"
			"	background: qlineargradient(x1: 0, y1: 0.5, x2: 1.5, y2: 0.5, stop: 0 green, stop: 1 white);"
			"}"
			"QProgressBar::chunk#BatStat[batterystatus=\"discharging\"] {"
			"	background: qlineargradient(x1: 0, y1: 0.5, x2: 1.5, y2: 0.5, stop: 0 red, stop: 1 white);"
			"}";
	batteryStatus->setStyleSheet(batStyleSheet);

	/* add all to the main layout */
	QHBoxLayout* bottom = new QHBoxLayout();
	bottom->addLayout(setupButtons());
	bottom->addWidget(setupStats());
	QHBoxLayout* battery = new QHBoxLayout();
	battery->addWidget(new QLabel("Battery: "));
	battery->addWidget(batteryStatus);

	leftLayout->addWidget(currentGame, 14);
	leftLayout->addLayout(bottom, 4);
	leftLayout->addStretch(1);
	leftLayout->addLayout(battery, 2);
}

RTMainWindow::RTMainWindow(GameConfiguration& config, QWidget* parent) :
		QMainWindow(parent), popupTimer(NULL), popupNeedsRebuilding(true)
{
	setupButtons();
	setupToolbar();
	setupLayout(config);
	setWindowIcon(QIcon(QCoreApplication::applicationDirPath() +"/../img/logo.png"));
	setWindowTitle(QString("RoboTower GUI"));
	QObject::connect(startBtn, SIGNAL(clicked()), this, SLOT(startOnClick()));
	QObject::connect(stopBtn, SIGNAL(clicked()), this, SLOT(stopOnClick()));
	QObject::connect(newGameAction, SIGNAL(triggered()), this,
			SLOT(newGameClicked()));
	QObject::connect(currentGame, SIGNAL(togglePause()), this,
			SIGNAL(togglePause()));
	QFont font(this->font());
	font.setBold(true);
	font.setPointSize(font.pointSize() + 2);
	this->setFont(font);

}

void RTMainWindow::startOnClick()
{
	setButtonStatus(true);
	emit start();
}

void RTMainWindow::stopOnClick()
{
	popupNeedsRebuilding = true;
	setButtonStatus(false);
	emit stop();
}

void RTMainWindow::newGameClicked()
{
	setButtonStatus(false);
	emit newGame();
}

void RTMainWindow::setButtonStatus(bool isRunning)
{
	currentGame->setPauseEnabled(isRunning);
	startBtn->setEnabled(!isRunning);
	stopBtn->setEnabled(isRunning);
}

void RTMainWindow::updateCardStatus(int cardNumber, bool status)
{
	cardsLayout->setCardStatus(cardNumber, status);
	if (status)
		soundmanager.play(SoundManager::Recharged);
	else
		soundmanager.play(SoundManager::Trapped);
}

void RTMainWindow::updateTime(int timeToLive)
{
	currentGame->updateTimer(timeToLive);
	if (timeToLive == 0)
	{
		soundmanager.play(SoundManager::Win);
		QMessageBox message;
		message.setText("Hai Battuto il Robot!");
		message.exec();
	}
}

void RTMainWindow::updatePoints(int score)
{
	currentGame->updateScore(score);
}

void RTMainWindow::updateTowers(int factoryNumber, int towersNumber)
{
	int oldFactoriesNumber = currentGame->getFactoriesNumber();
	currentGame->updateCounter(towersNumber, factoryNumber);
	if (towersNumber == 0)
	{
		soundmanager.play(SoundManager::Lose);
		QMessageBox message;
		message.setText(QString::fromUtf8("Il robot ha abbattuto la torre! Tutto è perduto!"));
		message.exec();
	}
	else if(factoryNumber < oldFactoriesNumber)
	{
		soundmanager.play(SoundManager::Factory);
	}
}

void RTMainWindow::updateHistory(int won, int lost, int score)
{
	setButtonStatus(false);
	statWon->setText(QString::number(won));
	statLost->setText(QString::number(lost));
	statTotalScore->setText(QString::number(score));
}

void RTMainWindow::updateBatteryStatus(int newStatus)
{
	QPalette palette = this->palette();
	if(newStatus < 0) {
		newStatus *= -1;
		batteryStatus->setProperty("batterystatus", "charging");
	} else {
		batteryStatus->setProperty("batterystatus", "discharging");
	}
	batteryStatus->setStyleSheet(batteryStatus->styleSheet());
	batteryStatus->setValue(newStatus);

}

void RTMainWindow::updateSetupPopup(int remainingTime)
{
	if (popupNeedsRebuilding)
	{
		delete popupTimer;
		buildSetupPopup();
	}
	popupTimer->update(remainingTime);
	if (remainingTime == 0)
	{
		popupNeedsRebuilding = true;
		delete popupTimer; // maybe this isn't necessary anymore...
		popupTimer = NULL;
		soundmanager.play(SoundManager::Start);
	}
}

void RTMainWindow::buildSetupPopup()
{
	const int popupWidth = 200;
	const int popupHeight = 60;
	int xAlign = x() + width() / 2 - popupWidth / 2;
	int yAlign = y() + height() / 2 - popupHeight / 2;
	popupTimer = new RTPopupTimer();
	popupNeedsRebuilding = false; // ok, we have it!
	popupTimer->setWindowModality(Qt::ApplicationModal);
	popupTimer->setGeometry(xAlign, yAlign, popupWidth, popupHeight);
	popupTimer->setWindowTitle("Starting...");
	popupTimer->setFont(font());
	popupTimer->show();
	/* make close the popup shut down the entire application */
	QObject::connect(popupTimer, SIGNAL(closed()), this, SLOT(stopOnClick()));
}

RTMainWindow::~RTMainWindow()
{
	delete popupTimer;
}
