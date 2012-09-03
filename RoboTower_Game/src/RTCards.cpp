/*,
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

void RTCards::addCards(GameConfiguration& config)
{
	int na = config.getNumActions();
	for (int i = 0; i < na; i++)
	{
		std::vector<ConfigRfidEntry> rfidList = config.getRfidList(i);
		for (size_t j = 0; j < rfidList.size(); j++)
		{
			int cardNumber = rfidList.at(j).num;
			QString action = QString::fromStdString(rfidList.at(j).action);
			RTCard* card = new RTCard(cardNumber, action);
			addWidget(card, j, i, 1, 1);
			cardMap[cardNumber] = card;
		}

	}
}

RTCard::RTCard(int number, QString action) :
		QLabel()
{
	const int borderWidth = 2;
	num = number;
	image.load("../img/actions/" + action + ".png");
	setScaledContents(true);
	setFrameStyle(borderWidth);
	setCardStatus(true);
	setMinimumSize(QSize(40, 60));
}

void RTCard::setCardStatus(bool isActive)
{
	if (isActive)
	{
		setPixmap(QPixmap::fromImage(image));
	}
	else
	{
		QImage img(image);
		QRgb col;
		int gray;
		int width = img.width();
		int height = img.height();
		for (int i = 0; i < width; ++i)
		{
			for (int j = 0; j < height; ++j)
			{
				col = img.pixel(i, j);
				gray = qGray(col);
				img.setPixel(i, j, qRgb(gray, gray, gray));
			}
		}
		setPixmap(QPixmap::fromImage(img));
	}
}

void RTCard::paintEvent(QPaintEvent * event)
{
	QString number = QString::number(num);
	QLabel::paintEvent(event);
	QPainter p(this);
	p.setPen(QColor::fromRgb(0, 0, 0, 255));
	/* set the font size relative to the card width */
	QFont f = p.font();
	f.setPointSize(geometry().width() / 6.3);
	p.setFont(f);
	register int textWidth =  p.fontMetrics().width(number);
	register int textHeight = p.fontMetrics().height();
	/* (123px + 57px/2) / 180px = 0.841667
	 * where 57px is the card base (the green area) when card height = 180px
	 * and 123 = 180 - 57 is the drawing height */
	register int verticalPos = 0.841667 * geometry().height() - textHeight / 2;
	register int horizontalPos = (geometry().width() - textWidth) / 2;
	p.drawText(QRect(horizontalPos, verticalPos,
			textWidth, textHeight), Qt::AlignCenter, number);
}

