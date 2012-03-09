/***************************************************************************
                          fuzzy_set.cpp  -  description
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
//      Module: fuzzy_set
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.


#include "fuzzy_set.h"
#include "math.h"

// Class fuzzy_set 

fuzzy_set::fuzzy_set()
{
  label=NULL;
}

fuzzy_set::fuzzy_set(const fuzzy_set &right)
{
  label=(char *) malloc(strlen(right.label)+1);
  strcpy(this->label,right.label);
}

fuzzy_set::fuzzy_set (char* alabel)
{
  label=(char *) malloc(strlen(alabel)+1);
  strcpy(label,alabel);
}

fuzzy_set::~fuzzy_set()
{
  if (label != NULL) free(label);
}

