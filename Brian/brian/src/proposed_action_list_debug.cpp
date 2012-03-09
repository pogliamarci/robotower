/***************************************************************************
                          proposed_action_list_debug.cpp  -  description
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

#include "proposed_action_list_debug.h"
#include <iostream>

void viewpacl (proposed_action_list * pal)
{

  proposed_action_list::iterator i=pal->begin();

  cout<<"\n--------------------- PRINTING PROPOSED ACTION LIST ----------------------\n";
    fflush(stdout);

    cout<<"\nLenght of proposed action list = ";
    cout<<pal->size();
    
  int num = 0;

  while(i!=pal->end())
    {
     cout<<"\ni = ";
     cout<<num;
     cout<<"\nName = ";
     cout<<(*i).second->get_name();
     cout<<"\nLabel = ";
     cout<<(*i).second->get_label();
     cout<< "\nMembership value = ";
     cout<<(*i).second->get_membership_value();

    	i++;
      num++;
      fflush(stdout);
    }
}
