/***************************************************************************
                          shape.cpp  -  description
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
//      Module: shape
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "shape.h"
#include "destroy_object.h"
#include <math.h>

// Class shape 

shape::shape()
  :fuzz_set_multimap()
{
  label=NULL;
}

shape::shape(const shape &right)
  :fuzz_set_multimap(right)
{
  label=(char *) malloc(strlen(right.label)+1);
  strcpy(this->label,right.label);
}

shape::shape (const char* alabel)
  :fuzz_set_multimap()
{
  label=(char *) malloc(strlen(alabel)+1);
  strcpy(label,alabel);
}


shape::~shape()
{
  for_each(begin(),end(),destroy_object<fuzzy_set> ());
  if(label != NULL) free(label);
}

//## Other Operations (implementation)

fuzzy_set * shape::get_set (const char* label)
{
  iterator i = find(label);
  // find() return an iterator to element whose key is name; we obtain a pointer to that object by
  // referencing this iterator; this pointer is typecasted to crisp_data.
  if (i==end())
    return NULL;
  else
    return (fuzzy_set *) (*i).second;
}

void shape::add_set (fuzzy_set *aset)
{
  insert(pair<const char *, fuzzy_set *>(aset->get_label(),aset));
}


