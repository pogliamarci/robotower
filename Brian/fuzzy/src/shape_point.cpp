/***************************************************************************
                          shape_point.cpp  -  description
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
//      Module: shape_point
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.


#include "shape_point.h"

#include <math.h>

// Class shape_point 

shape_point::shape_point()
  :point(),xa(0),ya(0),xb(0),yb(0),xc(0),yc(0),xd(0),yd(0)
{
}

shape_point::shape_point(const shape_point &right)
  :point(right),xa(0),ya(0),xb(0),yb(0),xc(0),yc(0),xd(0),yd(0)
{
}

shape_point::shape_point(float axa, float aya, float axb, float ayb, float axc, float ayc, float axd, float ayd)
  :point()
{
  xa=axa;
  ya=aya;
  xb=axb;
  yb=ayb;
  xc=axc;
  yc=ayc;
  xd=axd;
  yd=ayd;
}

shape_point::~shape_point()
{
}

