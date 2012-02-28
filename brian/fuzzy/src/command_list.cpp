/***************************************************************************
                          command_list.cpp  -  description
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



#include "command_list.h"
#include "command_multimap.h"

// Class command_list

command_list::command_list()
{
}

command_list::command_list(const command_list& right)
  :command_multimap(right) {}

command_list::~command_list()
{
}


command * command_list::get_command (const char *label)
{
  iterator i;
  // find() return an iterator to element whose key is name; we obtain a pointer to that object by
  // referencing this iterator; this pointer is typecasted to command.
  i=find(label);
  if (i==end())
    return NULL;
  else
    return (*i).second;
}

void command_list::add (command *acommand)
{
  // add an element of kind command into the multimap
  insert(pair<const char *, command *>(acommand->get_label(),acommand));
}

