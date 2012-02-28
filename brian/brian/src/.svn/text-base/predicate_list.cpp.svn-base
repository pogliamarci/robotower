/***************************************************************************
                          predicate_list.cpp  -  description
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


#include "predicate_list.h"

predicate_list::predicate_list():predicate_multimap(){
}

predicate* predicate_list::get(const char *name)
{
 iterator i;
  // find() return an iterator to element whose key is name; we obtain a pointer to that object by
  // referencing this iterator; this pointer is typecasted to pred_beh_parent.
  if (name[0] != '*')
    {
      i=find(name);
      if (i==end())
	return NULL;
      else
	return (*i).second;
    }
  else
    {
      i = begin();
      while (i != end())
	{
	  if (strstr((*i).first,name+1) != 0)
	    {
	      return (*i).second;
	    }
	  i++;
	}
      return NULL;
    }
}

void predicate_list::add(predicate * apredicate)
{
  // Modified to avoid several copies of the same predicate
  const char* pred_name = apredicate->get_name();
   multimap<const char*, predicate *>::iterator it = find(pred_name);
   if (it != end())
     {
       it->second->set_value(apredicate->get_value());
       it->second->set_reliability(apredicate->get_reliability());
       delete apredicate;
     }
   else
     {
       insert(pair<const char*, predicate *>(pred_name, apredicate));
     }
}


predicate_list::predicate_list(const predicate_list &right):predicate_multimap(right){
}


predicate_list  predicate_list::operator = (predicate_list &p) {
  
  predicate_list::iterator i= p.begin();
  while (i != p.end()) {
    this->add((*i).second);
    i++;
  }
  return *this;
}

predicate_list::~predicate_list()
{
/*
  for (predicate_list::iterator it = begin(); it != end(); ++it)
  {
    delete it->second;
  }
*/
}

void predicate_list::delete_proposed()
{
  vector<iterator> to_delete;
  for (iterator i = begin(); i != end(); i++)
    {
      if (strstr(i->first,"Proposed"))
	{
	  to_delete.push_back(i);
	}
    }

  for (unsigned int j = 0; j < to_delete.size(); j++)
    {
      delete to_delete[j]->second;
      erase(to_delete[j]);
    }
}
