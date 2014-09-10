/***************************************************************************
                          action_list_debug.cpp  -  description
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

#include "action_list_debug.h"
#include <iostream>

void viewacl (action_list * acl)
{
  action_list::iterator i=acl->begin();

  //cout<<"\n------------------------- PRINTING ACTION LIST ------------------------------\n";
  fflush(stdout);

  //cout<<"Lenght of action list = ";
  cout<<acl->size();

  int num = 0;

  /*while(i!=acl->end())
    {
      cout<<"\ni = ";
      cout<<num;
      cout<<"\nName = ";
      cout<<(*i).second->get_name();
      cout<<"\nLabel = ";
      cout<<(*i).second->get_label();
      cout<<"\nMembership value = ";
      cout<<(*i).second->get_membership_value();
      i++;
      num++;
    }*/
}
