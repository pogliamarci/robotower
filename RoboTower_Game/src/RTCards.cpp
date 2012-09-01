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
#include <iostream>
#include <vector>

void RTCards::addCards(GameConfiguration& config)
{
	int na = config.getNumActions();
	for (int i = 0; i < na; i++)
	{
		// std::string action = config.getAction(i);
		std::vector<ConfigRfidEntry> rfidList = config.getRfidList(i);
		for (size_t j = 0; j < rfidList.size(); j++)
		{
			int cardNumber = rfidList.at(j).num;
			RTCard* card = new RTCard(cardNumber);
			addWidget(card, j, i, 1, 1);
			cardMap[cardNumber] = card;
		}

	}
}

RTCards::RTCards(GameConfiguration& config) :
		QGridLayout()
{
	addCards(config);
	setAlignment(Qt::AlignTop);
}

void RTCards::setGeometry(const QRect& rect)
{
	const float aspectRatioFactor = 1.5;

	QGridLayout::setGeometry(rect);

	if (spacing() < 0)
		setSpacing(5);

	int w = rect.width() / columnCount() - spacing();
	int h = rect.height() / rowCount() - spacing();

	if (h < aspectRatioFactor * w)
		w = h / aspectRatioFactor;
	else
		h = aspectRatioFactor * w;

	int squareWidth = (w + spacing()) * columnCount() - spacing();
	int squareHeight = (h + spacing()) * rowCount() - spacing();
	int rectStartx = (rect.width() - squareWidth) / 2 + rect.x();
	int rectStarty = (rect.height() - squareHeight) / 2 + rect.y();

	for (int i = 0; i < rowCount(); i++)
	{
		for (int j = 0; j < columnCount(); j++)
		{
			QLayoutItem* item = this->itemAtPosition(i, j);
			if (item != NULL)
			{
				QRect geom(rectStartx + j * (w + spacing()),
						rectStarty + i * (h + spacing()), w, h);
				item->setGeometry(geom);
			}
		}
	}
}

void RTCards::setCardStatus(int cardNumber, bool cardStatus)
{
	cardMap[cardNumber]->setCardStatus(cardStatus);
}

void RTCard::setTextWhite()
{
	QPalette palette(this->palette());
	palette.setBrush(QPalette::Active, QPalette::WindowText, Qt::white);
	palette.setBrush(QPalette::Inactive, QPalette::WindowText, Qt::white);
	this->setPalette(palette);
	this->setAlignment(Qt::AlignCenter);
	QFont font(this->font());
	font.setBold(true);
	font.setPointSize(font.pointSize() + 2);
	this->setFont(font);
}

RTCard::RTCard(int number) :
		QLabel()
{
	const int borderWidht = 2;
	QString labelText = QString::number(number);
	setText(labelText);
	setTextWhite();
	setFrameStyle(borderWidht);
	setCardStatus(true);
	setMinimumSize(QSize(40, 60));
}

void RTCard::setCardStatus(bool isActive)
{
	QColor color = (isActive) ? Qt::blue : Qt::red;
	QPalette Pal(this->palette());
	Pal.setColor(QPalette::Window, color);
	setPalette(Pal);
	setAutoFillBackground(true);
}

