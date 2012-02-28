/***************************************************************************
                          association.cpp  -  description
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
//      Module: association
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "association.h"

#include <math.h>

// Class association 

association::association()
{
  label=NULL;
  shape=NULL;
}

association::association(const association &right)
{
  label=(char *) malloc(strlen(right.label)+1);
  shape=(char *) malloc(strlen(right.shape)+1);
  strcpy(this->label,right.label);
  strcpy(this->shape,right.shape);
}

association::association(const char* alabel,const char* ashape)
{
  label=(char *) malloc(strlen(alabel)+1);
  shape=(char *) malloc(strlen(ashape)+1);
  strcpy(label,alabel);
  strcpy(shape,ashape);
}

association::~association()
{
  if (label != NULL) free(label);
  if (shape != NULL) free(shape);
}

