/***************************************************************************
                          singleton_defuzzyfier.cpp  -  description
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
//      Module: singleton_defuzzyfier
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.



#include "singleton_defuzzyfier.h"
#include <math.h>

// Class singleton_defuzzyfier 

#define GRAD2RAD 0.01745

singleton_defuzzyfier::singleton_defuzzyfier()
  :defuzzyfier()
{
}

singleton_defuzzyfier::singleton_defuzzyfier(const singleton_defuzzyfier &right)
  :defuzzyfier(right)
{
}

singleton_defuzzyfier::singleton_defuzzyfier (shape_file_parser *shapefileparser, assoc_file_parser *assocfileparser)
  :defuzzyfier(shapefileparser,assocfileparser)
{
}


singleton_defuzzyfier::~singleton_defuzzyfier()
{
}

//## Other Operations (implementation)

command_list * singleton_defuzzyfier::defuzzyfy (action_list * actionlist)
{
  shapes_list * sl = get_shapelist();
  association_list * al = get_assoc();
  command_list * list = new command_list();
  
  action_list::iterator i = actionlist->begin();
  while (i != actionlist->end()) 
    {
      float setpoint=0,sum=0;
      float setpoint_x=0,setpoint_y=0;
      shape * curr;      

      const char * name = (*i).second->get_name();                              // get the name of the action from the actionlist
      const char * tempshape = (al->get_association(name))->get_shape();        // get the name of the shape from the association
      curr = sl->get_shape(tempshape);                                          // get the shape from the shapelist
      
      pair<action_list::iterator,action_list::iterator> p=actionlist->equal_range(name); 
      // get the begin and the end of the action with the same name in the actionlist 
 
      while(p.first != p.second)
	{    
	  float def_value,mem_value;
	  fuzzy_set * cset;

	  const char * templabel = (*(p.first)).second->get_label();      // get the label from the actionlist
	  cset = curr->get_set(templabel);                                // get the current set 
	  def_value=(*(singleton *)cset).get_a();                         // get the defuzzy value
	  mem_value=(*(p.first)).second->get_membership_value();          // get the membership value
	  // Changed by mr:060216: for polar coordinates
	  if (0 != strstr(name,"Angle"))
	    {
	      setpoint_x += cosf(def_value*GRAD2RAD)*mem_value; 
	      setpoint_y += sinf(def_value*GRAD2RAD)*mem_value; 
	    }
	  else
	    {
	      setpoint += def_value*mem_value; 
	    }

	  // Changed by mr:280603:
	  sum=sum+mem_value;
	  p.first++;      
	}

      // Changed by mr:060216: for polar coordinates
      if (0 != strstr(name,"Angle"))
	{
	  setpoint = atan2f(setpoint_y,setpoint_x)/GRAD2RAD;
	  if (setpoint < 0)
	    {
	      setpoint += 360;
	    }
	}
      else
	{
	  setpoint /= sum;
	}

      i = p.second;       
      if (sum != 0) list->add(new command(name,setpoint));
    }
  return list;
  }


