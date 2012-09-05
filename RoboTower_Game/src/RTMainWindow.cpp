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

#include <iostream>

void RTMainWindow::setupToolbar()
{
	fileToolBar = addToolBar(tr("MainBar"));
	newGameAction = new QAction(tr("&New game"), this);
	fileToolBar->addAction(newGameAction);
	addToolBar(Qt::TopToolBarArea, fileToolBar);
}

void RTMainWindow::setupButtons()
{
	btnLayout = new QVBoxLayout();
	startBtn = new QPushButton("Start");
	stopBtn = new QPushButton("Stop");
	btnLayout->addWidget(startBtn);
	btnLayout->addWidget(stopBtn);
	stopBtn->setEnabled(false);
}

void RTMainWindow::setupStats()
{
	statsGroupBox = new QGroupBox();
	statsLayout = new QGridLayout();
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
}

void RTMainWindow::setupLayout(GameConfiguration& config)
{
	mainWidget = new QWidget();
	mainLayout = new QHBoxLayout();
	leftLayout = new QGridLayout();
	setCentralWidget(mainWidget);

	/* Children widgets */
	currentGame = new RTCurrentGameWidget();
	cardsLayout = new RTCards(config);

	mainWidget->setLayout(mainLayout);
	mainLayout->addLayout(leftLayout);
	mainLayout->addLayout(cardsLayout);

	/* add all to the main layout */
	leftLayout->addWidget(currentGame, 1, 1, 2, 4);
	leftLayout->addLayout(btnLayout, 3, 1, 1, 2);
	leftLayout->addWidget(statsGroupBox, 3, 3, 1, 2);
}

RTMainWindow::RTMainWindow(GameConfiguration& config, QWidget* parent) :
		QMainWindow(parent), popupTimer(NULL)
{
	setupButtons();
	setupStats();
	setupToolbar();
	setupLayout(config);
	setWindowIcon(QIcon("../img/logo.png"));
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
}

void RTMainWindow::updateTime(int timeToLive)
{
	currentGame->updateTimer(timeToLive);
}

void RTMainWindow::updatePoints(int score)
{
	currentGame->updateScore(score);
}

void RTMainWindow::updateTowers(int factoryNumber, int towersNumber)
{
	currentGame->updateCounter(towersNumber, factoryNumber);
}

void RTMainWindow::updateHistory(int won, int lost, int score)
{
	setButtonStatus(false);
	statWon->setText(QString::number(won));
	statLost->setText(QString::number(lost));
	statTotalScore->setText(QString::number(score));
}

void RTMainWindow::updateSetupPopup(int remainingTime)
{
	if (popupTimer == NULL)
	{
		buildSetupPopup();
	}
	popupTimer->update(remainingTime);
	if (remainingTime == 0)
	{
		delete popupTimer;
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
	popupTimer->setWindowModality(Qt::ApplicationModal);
	popupTimer->setGeometry(xAlign, yAlign, popupWidth, popupHeight);
	popupTimer->setWindowTitle("Starting...");
	popupTimer->setFont(font());
	popupTimer->show();
}

RTMainWindow::~RTMainWindow()
{
	delete popupTimer;
}
