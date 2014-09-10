/***************************************************************************
                          triangle.cpp  -  description
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
//      Module: triangle
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "triangle.h"

#include <math.h>

// Class triangle 

triangle::triangle()
  :trapezium()
{
}

triangle::triangle(const triangle &right)
  :trapezium(right)
{
}

triangle::triangle (char* alabel, float aa, float ab, float ad)
  :trapezium(alabel,aa,ab,ab,ad)
{
}

triangle::~triangle()
{
}


