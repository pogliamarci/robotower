/***************************************************************************
                          div_triangle.cpp  -  description
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
//      Module: div_triangle
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.


#include "div_triangle.h"

#include <math.h>

// Class div_triangle 

div_triangle::div_triangle()
  :trapezium()
{
}

div_triangle::div_triangle(const div_triangle &right)
  :trapezium(right)
{
}

div_triangle::div_triangle (char* alabel, float aa, float ab, float ac, float ad)
   :trapezium(alabel,ac,ad,aa,ab)
{
}


div_triangle::~div_triangle()
{
}

