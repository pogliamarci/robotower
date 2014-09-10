/***************************************************************************
                          association_list.cpp  -  description
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
//      Module: association_list
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.


#include "association_list.h"
#include "destroy_object.h"


#include <math.h>


// Class association_list 

association_list::association_list()
  :association_set_multimap()
{
}

association_list::association_list(const association_list &right)
  :association_set_multimap(right)
{
}


association_list::~association_list()
{
  for_each(begin(),end(),destroy_object<association> ());
}

//## Other Operations (implementation)

association* association_list::get_association (const char* alabel)
{
  iterator i = find(alabel);
  // find() return an iterator to element whose key is name; we obtain a pointer to that object by
  // referencing this iterator; this pointer is typecasted to crisp_data.
  if (i==end())
    return NULL;
  else
    return (association *) (*i).second;
}

void association_list::add_association (association *aassoc)
{
  // add an element of kind fuzzy_set into the multimap
  insert(pair<const char *, association *>(aassoc->get_label(),aassoc));
}

