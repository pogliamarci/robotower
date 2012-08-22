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

void RTCards::setRowsAndColsDim()
{
	for (int i = 0; i < ROWS; i++)
	{
		this->setRowMinimumHeight(i, 60); // TODO
	}
	for (int i = 0; i < COLS; i++)
	{
		this->setColumnMinimumWidth(i, 30); // TODO
	}
}

void RTCards::addCards()
{
	for (int i = 0; i < ROWS * COLS; i++)
	{
		RTCard* card = new RTCard(i);
		//cardGrid->addWidget(card, i % ROWS, i / ROWS, 1, 1, Qt::AlignCenter);
		this->addWidget(card, i % ROWS, i / ROWS, 1, 1); // TODO
		cardList.push_back(card);
	}
}

RTCards::RTCards() :
		QGridLayout()
{
	//abel = new QLabel("Active Cards");
	//addWidget(label);
	//cardGrid = new QGridLayout();
	addCards();
	setRowsAndColsDim();
	//addLayout(cardGrid);
	setAlignment(Qt::AlignTop);
}

void RTCards::setGeometry(const QRect& rect)
{
	QGridLayout::setGeometry(rect);

	int w = rect.width() / columnCount() - spacing();
	int h = rect.height() / rowCount() - spacing();

	if(h<1.5*w) w = h/1.5;
	else h = 1.5*w;

	int squareWidth = (w+spacing())*columnCount() - spacing();
	int squareHeight = (h+spacing())*rowCount() - spacing();
	int rectStartx = (rect.width() - squareWidth) / 2 + rect.x();
	int rectStarty = (rect.height() - squareHeight) / 2 + rect.y();

	for(int i=0; i < rowCount(); i++) {
		for(int j=0; j<columnCount(); j++) {
			QLayoutItem* o = this->itemAtPosition(i,j);
			QRect geom(rectStartx + j*(w+spacing()), rectStarty + i*(h+spacing()), w, h);
			o->setGeometry(geom);
		}
	}
}

void RTCards::setCardStatus(int cardNumber, bool cardStatus)
{
	cardList[cardNumber]->setCardStatus(cardStatus);
}

void RTCard::setTextWhite()
{
	QPalette palette(this->palette());
	palette.setBrush(QPalette::Active, QPalette::WindowText, Qt::white);
	this->setPalette(palette);
	this->setAlignment(Qt::AlignCenter);
	QFont font(this->font());
	font.setBold(true);
	font.setPointSize(font.pointSize()+2);
	this->setFont(font);
}

RTCard::RTCard(int number) : QLabel()
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
}

