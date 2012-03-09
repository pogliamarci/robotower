/***************************************************************************
                          action_list.cpp  -  description
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


#include "action_list.h"
#include "action_multimap.h"

// Class action_list

action_list::action_list()
			 :action_multimap()
{
}

action_list::action_list(const action_list& right)
  :action_multimap(right) {}

action_list::~action_list()
{
}


action_list * action_list::get_by_name (const char *name)
{
  // find all elements whose name is "name"
  pair <iterator,iterator> p=equal_range(name);
  // create the object to be returned
  action_list *a= new action_list();
  // add a copy of each element found in the new object
  while (p.first!=p.second)
    {
      a->insert(pair<const char *, action *>( (*(p.first)).second->get_label() ,(*(p.first)).second) );
      p.first++;
    }
  // exit
  return a;
}

action * action_list::get_by_name_label (const char *name, const char *label)
{
  // gets all the actions whose name is "name"
  action_list *al = get_by_name(name);
  // search between them wich one has label "label"
  iterator i=al->find(label);
  // return value
  action * a=NULL;
  if (i!=al->end())
    // found
    a=(*i).second;
  //frees memory
  delete al;
  return a;
}

void  action_list::add (action *anaction)
{
  // add an element of kind action into the multimap
  insert(pair<const char *, action *>(anaction->get_name(),anaction));
}

