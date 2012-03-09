/***************************************************************************
                          crisp_data_list.cpp  -  description
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
//	This module implements crisp_data_list.h

#include "crisp_data_list.h"

// Class crisp_data_list

crisp_data_list::crisp_data_list()
  :crisp_data_multimap()
{
}

crisp_data_list::crisp_data_list(const crisp_data_list& right)
  :crisp_data_multimap(right)
{
}

crisp_data_list::~crisp_data_list()
{
}

crisp_data * crisp_data_list::get_by_name (const char *name)
{
  iterator i;
  // find() return an iterator to element whose key is name; we obtain a pointer to that object by
  // referencing this iterator; this pointer is typecasted to crisp_data.
  i=find(name);
  if (i==end())
    return NULL;
  else
    return (*i).second;
}

void crisp_data_list::add (crisp_data *acrispdata)
{
  // add an element of kind crip_data into the multimap
  insert(pair<const char *, crisp_data *>(acrispdata->get_name(),acrispdata));
}

//SM del
void crisp_data_list::del (crisp_data *acrispdata)
{
iterator i;
i=find(acrispdata->get_name());//becco l'iteratore al dato che cerco di cancellare
if (!(i==end())) erase(i); //elimino il dato crisp puntato da quell'iteratore, se esiste
}
