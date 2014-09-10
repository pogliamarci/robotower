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

#include "RTCurrentGameWidget.h"

#define PAUSE_STRING "&Pause"
#define RESUME_STRING "&Resume"

RTCurrentGameWidget::RTCounter::RTCounter(QString color)
{
	count = 0;
	image.load(
			QCoreApplication::applicationDirPath() + "/../img/counter/"
					+ color + ".png");
}

void RTCurrentGameWidget::RTCounter::updateCounter(int count)
{
	this->count = count;

	QLayoutItem* item;
	while ((item = takeAt(0)) != NULL)
	{
		delete item->widget();
		delete item;
	}

	for (int i = 0; i < count; i++)
	{
		QLabel* label = new QLabel();
		label->setPixmap(QPixmap::fromImage(image));
		addWidget(label);
	}
}

RTCurrentGameWidget::RTCurrentGameWidget(QWidget* parent) :
		QGroupBox(parent)
{
	isPaused = false;
	QVBoxLayout* theLayout = new QVBoxLayout();
	pauseBtn = new QPushButton(PAUSE_STRING);
	pauseBtn->setEnabled(false);
	currentScore = new QLCDNumber();
	currentTTL = new QLCDNumber();
	towersCnt = new RTCounter("red");
	factoriesCnt = new RTCounter("yellow");


	QHBoxLayout* innerGoalsLayout = new QHBoxLayout();

	setTitle("Current Game");

	/* inner goals layout */
	// innerGoalsLayout->addWidget(new QLabel("Towers: "), 1, 1, 1, 1);
	innerGoalsLayout->addLayout(towersCnt, 1);
	// innerGoalsLayout->addWidget(new QLabel("Factories: "), 2, 1, 1, 1);
	innerGoalsLayout->addLayout(factoriesCnt, 3);
	// innerGoalsLayout->setMinimumHeight(1, 70);


	/* main layout */
	theLayout->addWidget(pauseBtn);

	/* add score */
	QHBoxLayout* scoreLyt = new QHBoxLayout();
	scoreLyt->addWidget(new QLabel("Score:"));
	scoreLyt->addWidget(currentScore);
	theLayout->addLayout(scoreLyt);

	/* add time to live */
	QHBoxLayout* ttlLyt = new QHBoxLayout();
	ttlLyt->addWidget(new QLabel("Time to live:"));
	ttlLyt->addWidget(currentTTL);
	theLayout->addLayout(ttlLyt);

	/* add goals*/
	theLayout->addLayout(innerGoalsLayout);

	setLayout(theLayout);

	QObject::connect(pauseBtn, SIGNAL(clicked()), this, SLOT(onPauseClick()));
}

void RTCurrentGameWidget::setPauseEnabled(bool isPauseEnabled)
{
	pauseBtn->setEnabled(isPauseEnabled);
	if (isPauseEnabled)
	{
		pauseBtn->setText(PAUSE_STRING);
		isPaused = false;
	}
}

void RTCurrentGameWidget::onPauseClick()
{
	if (isPaused)
	{
		pauseBtn->setText(PAUSE_STRING);
		isPaused = false;
	}
	else
	{
		pauseBtn->setText(RESUME_STRING);
		isPaused = true;
	}
	emit togglePause();
}

void RTCurrentGameWidget::updateCounter(int towers, int factories)
{
	towersCnt->updateCounter(towers);
	factoriesCnt->updateCounter(factories);
}

void RTCurrentGameWidget::updateScore(int newScore)
{
	currentScore->display(newScore);
}

void RTCurrentGameWidget::updateTimer(int newTtl)
{
	currentTTL->display(newTtl);
}
