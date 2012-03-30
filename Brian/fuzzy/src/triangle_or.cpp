/***************************************************************************
                          triangle_or.cpp  -  description
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
//      Module: triangle_or
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "triangle_or.h"

#include "math.h"

// Class triangle_or 

triangle_or::triangle_or()
  :fuzzy_set(),a(0),b(0)
{
}

triangle_or::triangle_or(const triangle_or &right)
  :fuzzy_set(),a(0),b(0)
{
}

triangle_or::triangle_or (char* alabel, float aa, float ab)
  :fuzzy_set(alabel)
{
  a=aa;
  b=ab;
}


triangle_or::~triangle_or()
{
}

// Additional Declarations
  
float triangle_or::get_membership_value(float avalue)
{
  if (avalue >= b) return (1);
  else if (avalue > a && avalue < b) return((avalue-a)/(b-a));
  else return(0);
}


