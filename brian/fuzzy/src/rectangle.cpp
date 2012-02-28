/***************************************************************************
                          rectangle.cpp  -  description
                             -------------------
    begin                : Wed Sep 13 2000
    copyright            : (C) 2000 by Halva Giovanni & Giacomo
    email                : invehalv@airlab.elet.polimi.it
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
//      Module: rectangle
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "rectangle.h"

#include <math.h>

// Class rectangle 

rectangle::rectangle()
  :trapezium()
{
}

rectangle::rectangle(const rectangle &right)
  :trapezium(right)
{
}

rectangle::rectangle (char* alabel, float ab, float ac)
  :trapezium(alabel,ab,ab,ac,ac)
{
}


rectangle::~rectangle()
{
}

