/***************************************************************************
                          predicate_list_debug.cpp  -  description
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

#include "predicate_list_debug.h"
#include <iostream>

void viewpbpl(predicate_list * p)
{
  predicate_list::iterator i=p->begin();

  cout<<"\n--------------------- PRINTING PREDICATE LIST ----------------------\n";
  fflush(stdout);
  cout<<"\nLenght of predicate list = ";
  cout<<p->size();
  int num = 0;

  while(i!=p->end())
    {
      cout<<"\ni = ";
      cout<<num;
      cout<<"\nName = ";
      cout<<(*i).second->get_name();
      cout<<"\nMembership value = ";
      cout<<(*i).second->get_value();
      cout<<"\nReliability = ";
      cout<<(*i).second->get_reliability();
      i++;
      num++;
      fflush(stdout);
    }
}
