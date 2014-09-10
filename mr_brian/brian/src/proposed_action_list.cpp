/***************************************************************************
                          proposed_action_list.cpp  -  description
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


#include "proposed_action_list.h"




proposed_action_list::proposed_action_list ()
  :multimap<const char *, proposed_action *, ltstr >()
{
}

proposed_action_list::proposed_action_list (const proposed_action_list &right)
  :multimap<const char *, proposed_action *, ltstr >(right)
{
}

proposed_action_list::~proposed_action_list ()
{
}

proposed_action_list * proposed_action_list::get_by_name (const char *name)
{
  // find all elements whose name is "name"
  pair <iterator,iterator> p=equal_range(name);
  // create the object to be returned
  proposed_action_list *a= new proposed_action_list();
  // add a copy of each element found in the new object
  while (p.first!=p.second)
    {
      a->insert(pair<const char *, proposed_action *>( (*(p.first)).second->get_label() , (*(p.first)).second ) );
      p.first++;
    }
  // exit
  return a;
}

proposed_action_list * proposed_action_list::get_by_name_label (const char *name, const char *label)
{
  // gets all the actions whose name is "name"
  proposed_action_list *al = get_by_name(name);
  // search between them wich ones have label "label"
  pair <iterator,iterator> p=al->equal_range(label);
  // creates the variable to return
  proposed_action_list *result=new proposed_action_list();
  // put actions into result
  while (p.first!=p.second)
    {
      result->insert(pair<const char * , proposed_action *>( (*(p.first)).second->get_label() ,(*(p.first)).second ) );
      p.first++;
    }
  delete al;
  return result;
}

void  proposed_action_list::add (proposed_action *anaction)
{
  // add an element of kind action into the multimap

//   cout << "***************** Metodo ADD aggiungo: " << anaction->get_name() << " <=> " << anaction->get_label() << " proposta da: " << anaction->get_behavior_name() << '\n';
   
  insert(pair<const char *, proposed_action *>(anaction->get_name(),anaction));
}


void proposed_action_list::del (const char *name, const char *label)
{
  // delete all actions in the list whose name is 'name' and label is 'label'
  
  pair <iterator,iterator> p = this->equal_range(name);
  vector <iterator> vi;
  while (p.first != p.second)
    {
      if (strcmp(label, (*(p.first)).second->get_label()) == 0) {
	vi.push_back(p.first);
	delete (*(p.first)).second;
      }
      (p.first)++;
    }
  
  for(int i = vi.size() - 1; i >= 0; i--) {
    erase(vi[i]);
  }
}


void proposed_action_list::del (const char *name, const char *label, const char *behavior)
{
  // delete all actions in the list whose name is 'name' and label is 'label'
  
   pair <iterator,iterator> p = this->equal_range(name);
  vector <iterator> vi;
  while (p.first != p.second)
    {
      if (((label[0] == '*') && (strstr((*(p.first)).second->get_label(),label+1) != 0))
	  ||
	  (strcmp(label,"ANY") == 0)
	  ||
	  (strcmp(label, (*(p.first)).second->get_label()) == 0))
	{
	  if (strcmp(behavior, (*(p.first)).second->get_behavior_name()) != 0)
	    {
	      vi.push_back(p.first);
	      delete (*(p.first)).second;
	    }
	}
      (p.first)++;
    }
  
  for(int i = vi.size() - 1; i >= 0; i--) 
    {
      erase(vi[i]);
    }
}



proposed_action::proposed_action()
                : action()
{
  behavior_name=NULL;
}

proposed_action::proposed_action(const proposed_action& right)
  :action(right)
{
  this->behavior_name=(char *) malloc(strlen(right.behavior_name)+1);
  strcpy(this->behavior_name,right.behavior_name);
}

proposed_action::proposed_action (const char *aname, const char *alabel, const char * abehavior_name)
                :action(aname,alabel)
{
  behavior_name=(char *) malloc(strlen(abehavior_name)+1);
  strcpy(behavior_name,abehavior_name);;
}


proposed_action::proposed_action (const char *aname, const char *alabel, const char * abehavior_name, float am_f)
                :action(aname,alabel,am_f)
{
  behavior_name=(char *) malloc(strlen(abehavior_name)+1);
  strcpy(behavior_name,abehavior_name);;
}


proposed_action::~proposed_action()
{
  if (behavior_name!=NULL) free (behavior_name);
}

