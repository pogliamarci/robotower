/***************************************************************************
                          interf_obj.cpp  -  description
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
//	This module implements crisp_data.h

#include "crisp_data.h"

// Class crisp_data 

crisp_data::crisp_data()
      : data(), value(0)
{
}

crisp_data::crisp_data(const crisp_data& right)
  :data(right)
{
  this->value=right.value;
}

crisp_data::crisp_data (const char *aname, float avalue)
      : data(aname), value(0) 
{
  value=avalue;
}

crisp_data::crisp_data (const char *aname, float avalue, float areliability)
      : data(aname,areliability), value(0)
{
  value = avalue;
}


crisp_data::~crisp_data()
{
}
