/***************************************************************************
                          fuzzy_data_list_debug.cpp  -  description
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
//	This module implements fuzzy_data_list_debug.h

#include "fuzzy_data_list_debug.h"
#include <iostream>


void viewfdl(fuzzy_data_list * fdl)
{
  fuzzy_data_list::iterator w;

  cout<<"\n------------------------- PRINTING FUZZY DATA LIST -----------------------------\n";
  fflush(stdout);

  cout<<"\nLenght of fuzzy data list = ";
  cout<<fdl->size();

  int num = 0;

  for(w=fdl->begin();w!=fdl->end();w++)
    {
      cout<<"\ni = ";
      cout<<num;
      cout<<"\nName = ";
      cout<<(*w).second->get_name();
      cout<<"\nLabel = ";
      cout<<(*w).second->get_label();
      cout<<"\nMembership value = ";
      cout<<(*w).second->get_membership_value();
      num++;
      fflush(stdout);
    }
}

