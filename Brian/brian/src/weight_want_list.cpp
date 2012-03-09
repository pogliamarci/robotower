/***************************************************************************
                          weight_want_list.cpp  -  description
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


#include "weight_want_list.h"




// Class weight_want_list

weight_want_list::weight_want_list()
  :weight_want_multimap()
{
}

weight_want_list::weight_want_list(const weight_want_list &right)
  :weight_want_multimap(right)
{
}


weight_want_list::~weight_want_list()
{
}

float weight_want_list::get_weight (const char *name)
{
  iterator i;
  // find() return an iterator to element whose key is name; we obtain a pointer to that object by
  // referencing this iterator; this pointer is typecasted to crisp_data.
  i=find(name);
  if (i==end())
    return 0;
  else
    return (*i).second->get_value();
}

void weight_want_list::add (weight_want *aweightwant)
{
  // add an element of kind weight_want into the multimap
  insert(pair<const char *, weight_want *>(aweightwant->get_name(),aweightwant));
}

