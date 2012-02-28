/***************************************************************************
                          shapes_list.cpp  -  description
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
//      Module: shapes_list
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.

#include "shapes_list.h"
#include "destroy_object.h"
#include "math.h"

// Class shapes_list 

shapes_list::shapes_list()
  :shape_multimap()
{
}

shapes_list::shapes_list(const shapes_list &right)
  :shape_multimap(right)
{
}


shapes_list::~shapes_list()
{
  //Added by mr:021128
  shapes_list::iterator it;
  for (it = begin(); it != end(); it++)
    {
      delete it->first;
    }

  for_each(begin(),end(),destroy_object<shape> ());
}

//## Other Operations (implementation)

shape * shapes_list::get_shape(const char * shapename)
{
  iterator i = find(shapename);
  // find() return an iterator to element whose key is name; we obtain a pointer to that object by
  // referencing this iterator; this pointer is typecasted to crisp_data.
  if (i==end())
    return NULL;
  else
    return (shape *) (*i).second;
}

void shapes_list::add_shape (shape *ashape)
{
  insert(pair<const char *, shape *>(ashape->get_label(),ashape));
}


