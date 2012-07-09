/***************************************************************************
                          compose.cpp  -  description
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

// Compose
#include "compose.h"

proposed_action_list * mult_weight_composer::compose (weight_want_list *weights, proposed_action_list *actions)
{
  proposed_action_list::iterator i=actions->begin();
  for (;i!=actions->end();i++)
    {
      // for each action in the list, multiply its membership value with
      // weight value
      proposed_action *a=(*i).second;
      // retrieve behavior name
      const char *behav_name=a->get_behavior_name();
      // modify membership value
      a->set_membership_value(a->get_membership_value()*weights->get_weight(behav_name));
    }
  return actions;
}

proposed_action_list * max_weight_composer::compose (weight_want_list *weights, proposed_action_list *actions)
{
  // scroll the list
  proposed_action_list::iterator i=actions->begin();
  for (;i!=actions->end();i++)
    {
      // for each action in the list, set its membership value to the bigger
      // between itold value and the weight
      proposed_action *a=(*i).second;
      // retrieve behavior name
      const char *behav_name=a->get_behavior_name();
      // modify membership value
      float f=weights->get_weight(behav_name);
      if (f>a->get_membership_value())
	a->set_membership_value(f);
    }
  return actions;
}

proposed_action_list * min_weight_composer::compose (weight_want_list *weights, proposed_action_list *actions)
{
  // scroll the list
  proposed_action_list::iterator i=actions->begin();
  for (;i!=actions->end();i++)
    {
      // for each action in the list, set its membership value to the minimum
      // between itold value and the weight
      proposed_action *a=(*i).second;
      // retrieve behavior name
      const char *behav_name=a->get_behavior_name();
      // modify membership value
      float f=weights->get_weight(behav_name);
      if (f<a->get_membership_value())
	a->set_membership_value(f);
    }
  return actions;
}

float min_float_composer::compose(vector<float> *floatlist)
{
  vector<float>::iterator i=floatlist->begin();
  float ris=*i;
  for(;i!=floatlist->end();++i)
    if(*i<ris)
      ris=*i;
  return ris;
}


//ritorna il valore massimo da un vettore di float
float max_float_composer::compose(vector<float> *floatlist)
{
  vector<float>::iterator i=floatlist->begin();
  float ris=*i;
  for(;i!=floatlist->end();i++)
    if(*i>ris)
      ris=*i;
  return ris;
}


//ritorna la media
float average_float_composer::compose(vector<float> *floatlist)
{
  vector<float>::iterator i=floatlist->begin();
  float ris=0;
  for(i=floatlist->begin();i!=floatlist->end();i++)
    ris+=*i;
  return ris/floatlist->size();
}




//CLASSE COMPOSER
composer::composer()
{
}

composer::composer(weight_composer * aweight_composer, float_composer * afloat_composer)
{
  w_c=aweight_composer;
  f_c=afloat_composer;
}

composer::~composer()
{
  delete w_c;
  delete f_c;
}

action_list * composer::compose(weight_want_list *weights, proposed_action_list *actions)
{
  const char *actname=NULL, *actlabel=NULL;
  char *namelabel=NULL;
  // composes the actions with their weight by colling the weight_composer object
  action_list * acl = new action_list();
  proposed_action_list *pal=w_c->compose(weights,actions);
  // list of name and label processed
  // they are stored in the form strcat(name,label)
  set<char *, ltstr> * processed= new set<char *, ltstr>();
  // iterator used to scroll the list
  proposed_action_list::iterator i;
  // for every element in the list
  for(i=pal->begin();i!=pal->end();i++)
    {
      // retrieves name and label of the action beeing processed
      actname=(*i).second->get_name();
      actlabel=(*i).second->get_label();
      // unify name and label in a variable. An action has been processed if and only if his name+label is in set "processed" set
      namelabel=(char *) malloc(strlen(actname)+strlen(actlabel)+1);
      strcpy(namelabel,actname);
      strcat(namelabel,actlabel);
      // tests if the actions hadn't been processed
      if (processed->find(namelabel)==processed->end())
	{
	  // not yet processed

	  // retrieve all the actions with same name and label
	  proposed_action_list *subpal=pal->get_by_name_label(actname,actlabel);
	
	  // scroll them and find the composed value
	  proposed_action_list::iterator i1;
	  // v contains values to compose 
	  vector<float> *v=new vector<float>();
	  for (i1=subpal->begin();i1!=subpal->end();i1++)
	    // store the value in v
	    v->insert(v->end(),(*i1).second->get_membership_value());
	  	  
     // adds the action to the returned object	 
	  acl->add(new action(actname,actlabel,f_c->compose(v)));
	  	
	  // adds strcat(name,label) to the list of processed proposed actions.
	  processed->insert(namelabel);
	  // frees memory
	  delete subpal;
	  delete v;
	}
      else
	{
	  // already processed

	  // frees memory and goes to another object
	  if (namelabel!=NULL) free(namelabel);
	}
      free((char*)actname);
    }
  // frees memory from no-more-useful variables
  set<char *, ltstr>::iterator mi;
  for (mi=processed->begin();mi!=processed->end();mi++)
    free(*mi);
  delete processed;
  return acl;
}
