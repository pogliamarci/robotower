/***************************************************************************
                          fuzzy_data_list.cpp  -  description
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
//	This module implements fuzzy_data_list.h

#include "fuzzy_data_list.h"

// Class fuzzy_data_list

fuzzy_data_list::fuzzy_data_list ()
                :fuzzy_data_multimap()
{
}

fuzzy_data_list::fuzzy_data_list(const fuzzy_data_list& right)
  :fuzzy_data_multimap(right)
  {
  }
  


fuzzy_data_list::~fuzzy_data_list()
{
  //Added by mr:021128
  //for_each(begin(),end(),destroy_object<fuzzy_data>());
}

fuzzy_data_list * fuzzy_data_list::get_by_name (const char *name)
{
  // find all elements whose name is "name"
  pair <iterator,iterator> p=equal_range(name);
  // create the object to be returned
  fuzzy_data_list *f= new fuzzy_data_list();
  // add a copy of each element found in the new object
  while (p.first!=p.second)
    {
      f->insert(pair<const char *, fuzzy_data *>( (*(p.first)).second->get_label(),(*(p.first)).second) );
      p.first++;
    }
  // exit
  return f;
}

fuzzy_data * fuzzy_data_list::get_by_name_label (const char *name, const char *label)
{
  // gets all the fuzzy_data whose name is "name"
  fuzzy_data_list *fdl = get_by_name(name);
  // search between them wich one has label "label"
  iterator i=fdl->find(label);
  // return value
  fuzzy_data *f=NULL;
  if (i!=fdl->end())
    // found
    f=(*i).second;
  // frees memory
  //Added by mr:030422
  fdl->clear();
  delete fdl;
  return f;
}

void fuzzy_data_list::add (fuzzy_data *afuzzydata)
{
  // add an element of kind fuzzy_data into the multimap
  insert(pair<const char *, fuzzy_data *>(afuzzydata->get_name(),afuzzydata));
}

