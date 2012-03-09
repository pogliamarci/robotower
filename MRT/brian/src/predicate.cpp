/***************************************************************************
                          predicate.cpp  -  description
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
//	This module implements the objects used by other object
//	to interface each others.

#include "predicate.h"

// Class predicate

predicate::predicate()
      : value(0), reliability(1)
{
  name=NULL;
}

predicate::predicate(const predicate& right)
{
  this->name=(char *) malloc(strlen(right.name)+1);
  strcpy(this->name,right.name);
  this->value=right.value;
  this->reliability=right.reliability;
}

predicate::predicate (const char *aname, float avalue)
      : value(0), reliability(1)
{
  name=(char *) malloc(strlen(aname)+1);
  strcpy(name,aname);
  value=avalue;
}

predicate::predicate (const char *aname, float avalue, float areliability)
      : value(0), reliability(1)
{
  name=(char *) malloc(strlen(aname)+1);
  strcpy(name,aname);
  value=avalue;
  reliability=areliability;
}

predicate::~predicate()
{
  if (name!=NULL) free(name);
}
