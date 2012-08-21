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

#include "RTCards.h"
#include <string>

void RTCards::setRowsAndColsDim()
{
	for (int i = 0; i < ROWS; i++)
	{
		cardGrid->setRowMinimumHeight(i, 60);
		cardGrid->setRowStretch(i, 2);
	}
	for (int i = 0; i < COLS; i++)
	{
		cardGrid->setColumnMinimumWidth(i, 30);
		cardGrid->setColumnStretch(i, 1);
	}
}

void RTCards::addCards()
{
	for (int i = 0; i < ROWS * COLS; i++)
	{
		RTCard* card = new RTCard(i);
		cardGrid->addWidget(card, i % ROWS, i / ROWS, 1, 1, Qt::AlignCenter);
		cardList.push_back(card);
	}
}

RTCards::RTCards() :
		QVBoxLayout()
{
	label = new QLabel("Carte Attive");
	addWidget(label);
	cardGrid = new QGridLayout();
	addCards();
	setRowsAndColsDim();
	addLayout(cardGrid);
	setAlignment(Qt::AlignTop);
}

void RTCards::setCardStatus(int cardNumber, bool cardStatus)
{
	cardList[cardNumber]->setCardStatus(cardStatus);
}

void RTCard::setTextWhite()
{
	QBrush brush(QColor(255, 255, 255, 255));
	brush.setStyle(Qt::SolidPattern);
	QPalette palette(this->palette());
	palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
}

RTCard::RTCard(int number) :
		QLabel()
{
	const int borderWidht = 2;
	QString labelText = QString::number(number+1);
	setText(labelText);
	setTextWhite();
	setFrameStyle(borderWidht);
	setCardStatus(true);
}

void RTCard::setCardStatus(bool isActive)
{
	QColor color = (isActive) ? Qt::blue : Qt::red;
	QPalette Pal(this->palette());
	Pal.setColor(QPalette::Window, color);
	setPalette(Pal);
	setAutoFillBackground(true);
	setMinimumSize(QSize(30,60));
}

int RTCard::heightForWidth(int w) const
{
	return 2 * w;
}

