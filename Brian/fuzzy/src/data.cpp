/***************************************************************************
                          data.cpp  -  description
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
//	This module implements data.h

#include "data.h"

// Class data 

data::data()
      : reliability(1)
{
  name=NULL;
}

data::data(const data& right)
{
  this->name=(char *) malloc(strlen(right.name)+1);
  strcpy(this->name,right.name);
  this->reliability=right.reliability;
}

data::data (const char *aname)
      : reliability(1)
{
  name=(char *) malloc(strlen(aname)+1);
  strcpy(name,aname);
}

data::data (const char *aname, float areliability)
      : reliability(1)
{
  name=(char *) malloc(strlen(aname)+1);
  strcpy(name,aname);
  reliability=areliability;
}

data::~data()
{
  if (name!=NULL) free(name);
}
