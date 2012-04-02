/***************************************************************************
                          singleton.cpp  -  description
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
//      Module: singleton
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "singleton.h"

#include "math.h"

// Class singleton 

singleton::singleton()
  :fuzzy_set(),a(0)
{
}

singleton::singleton(const singleton &right)
  :fuzzy_set(right),a(0)
{
}

singleton::singleton (char* alabel, float aa)
  :fuzzy_set(alabel)
{
  a=aa;
}

singleton::~singleton()
{
}


// Additional Declarations

float singleton::get_membership_value(float avalue)
{
  if (avalue == a) return(1);
  else return(0);
}



