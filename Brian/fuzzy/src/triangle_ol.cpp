/***************************************************************************
                          triangle_ol.cpp  -  description
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
//      Module: triangle_ol
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "triangle_ol.h"

#include <math.h>


// Class triangle_ol 

triangle_ol::triangle_ol()
  :fuzzy_set(),c(0),d(0)
{
}

triangle_ol::triangle_ol(const triangle_ol &right)
  :fuzzy_set(right),c(0),d(0)
{
}

triangle_ol::triangle_ol (char* alabel, float ac, float ad)
  :fuzzy_set(alabel)
{
  c=ac;
  d=ad;
}


triangle_ol::~triangle_ol()
{
}


// Additional Declarations

float triangle_ol::get_membership_value(float avalue)
{
  if (avalue <= c) return(1);
  else if (avalue > c && avalue < d) return ((avalue-d)/(c-d));
  else return(0);
}  
