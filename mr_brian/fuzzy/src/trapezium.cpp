/***************************************************************************
                          trapezium.cpp  -  description
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
//      Module: trapezium
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "trapezium.h"

#include "math.h"

// Class trapezium 

trapezium::trapezium()
  :fuzzy_set(),a(0),b(0),c(0),d(0)
{
}

trapezium::trapezium(const trapezium &right)
  :fuzzy_set(right),a(0),b(0),c(0),d(0)
{
}

trapezium::trapezium (char* alabel, float aa, float ab, float ac, float ad)
  :fuzzy_set(alabel)
{
  a=aa;
  b=ab;
  c=ac;
  d=ad;
}


trapezium::~trapezium()
{
}

// Additional Declarations

float trapezium::get_membership_value(float avalue)
{
  if (avalue >= b && avalue <= c) return(1);
  else if (avalue > a && avalue <= b) return ((avalue-a)/(b-a));
  else if (avalue >= c && avalue < d) return((avalue-d)/(c-d));
  else return(0);
}


