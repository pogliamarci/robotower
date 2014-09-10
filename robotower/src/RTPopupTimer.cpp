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

#include "RTPopupTimer.h"
#include <iostream>

void RTPopupTimer::update(int remainingTime)
{
	text->setText(QString::number(remainingTime));
}

RTPopupTimer::RTPopupTimer(QWidget* parent) : QWidget(parent, Qt::Dialog)
{
	QFont f = this->font();
	f.setPointSize(f.pointSize() * 1.8);
	QLabel* description = new QLabel("Time to start: ");
	description->setFont(f);
	text = new QLabel("--");
	text->setFont(f);
	QHBoxLayout* lyt = new QHBoxLayout();
	lyt->addWidget(description);
	lyt->addWidget(text);
	setLayout(lyt);
}

void RTPopupTimer::closeEvent(QCloseEvent* e)
{
	emit closed();
}


void RTPopupTimer::keyPressEvent(QKeyEvent *e)
{
	if(e->key() == Qt::Key_Escape)
		close();
}

