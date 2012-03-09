/***************************************************************************
                          rules_behav.cpp  -  description
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

//This module defines and implements a rule behavior

#include "rules_behav.h"
#include <string>

#include "smdebug.h"

const float RELIABILITY_THRESHOLD = 0.1; // Rappresenta RhoLow, esclude tutti gli antecedenti con Reliability inferiore

rule::rule()
  :conditions(NULL),actionslist(NULL)
{
}

rule::rule(aggregation_tree * conds,proposed_action_list * acts)
{
  conditions=conds;
  actionslist=acts;
  ctm=new cando_tree_map();
}

rule::~rule()
{
  if (conditions!=NULL) delete conditions;
  if (actionslist!=NULL) delete actionslist;
  delete ctm;
}


/*VECCHIO METODO (SENZA FILTRO RELIABILITY)
proposed_action_list * rule::get_actions(predicate_list * predicates)
{
  // evaluating the conditions
  float mf=conditions->get_value(predicates,NULL);
  float rel=conditions->get_reliability(predicates,NULL);

  if (rel < RELIABILITY_THRESHOLD)
    {
      rel = 0.0;
    }

  // It continues only if  mf and rel are not 0
  if ((mf!=0.0)&&(rel!=0.0))
    {
      // it sets the membership_value fo mf*rel
      proposed_action_list::iterator i=actionslist->begin();
      while (i!=actionslist->end())
	{
	  (*i).second->set_membership_value(mf*rel);
	  i++;
	}
      return actionslist;
    }
  else {
    return NULL;
  }
}*/

proposed_action_list * rule::get_actions(predicate_list * predicates)
{
  // evaluating the conditions
  float mf=conditions->get_value(predicates,NULL);
  float rel=conditions->get_reliability(predicates,NULL);

  if (rel < RELIABILITY_THRESHOLD)//Esclude antecedenti con Reliability inferiore a RhoLow...
    {
      #ifdef DEBUG
	 		printf("\nAntecedente con mf=%f e rel=%f escluso",mf,rel);
	 	#endif
	   rel = 0.0;
    }

  // It continues only if  mf and rel are not 0
  if ((mf!=0.0)&&(rel!=0.0))//...e anche quelli con mf==0
    {
      #ifdef DEBUG
	 		printf("\n Antecedente con mf=%f e rel=%f considerato",mf,rel);
	 	#endif
	   // it sets the membership_value fo mf and reliability to rel
      proposed_action_list::iterator i=actionslist->begin();
      while (i!=actionslist->end())
	{
	  (*i).second->set_membership_value(mf);
	  (*i).second->set_reliability_value(rel);
	  i++;
	}
      return actionslist;
    }
  else {
    return NULL;
  }
}


rules_file_parser::rules_file_parser()
  :filename(NULL)
{
}

rules_file_parser::rules_file_parser(char * afilename)
{
  filename=(char *) malloc(strlen(afilename)+1);
  strcpy(filename,afilename);
}

rules_file_parser::~rules_file_parser()
{
  if (filename!=NULL) free(filename);
}

extern FILE *rulesin;
extern rules_list * parser();

rules_list * rules_file_parser::read_rules()
{
  if ((rulesin=fopen(filename, "r")) != NULL)
    {
      //if the file exists
      rules_list * rl=parser();
      fclose(rulesin);
      return rl;
    }
  else
    {
      //  it doesn't exist
      printf("FILE %s NOT FOUND\n",filename);
      return NULL;
    }
}

// class rules behavior
rules_behavior::rules_behavior(char * name,rules_file_parser * arules_file_parser)
  :behavior(name)
{
  rules_parser=arules_file_parser;
  rules=rules_parser->read_rules();
  // sets in each proposed action the behavior name attribute
  rules_list::iterator i=rules->begin();
  
  while (i!=rules->end())//per tutta la lista di regole...
    {
      //estraggo le azioni, per ogni azione della lista di azioni proposte, gli assegna il nome del behavior rispettivo
	   proposed_action_list * pal=(*i)->get_actionslist();
	   proposed_action_list::iterator y=pal->begin();
      while (y!=pal->end())
			{
	  		(*y).second->set_behavior_name(get_name());
	  		y++;
			}
      i++;
    }
}

rules_behavior::~rules_behavior()
{
  delete rules_parser;
  rules_list::iterator i=rules->begin();
  while (i!=rules->end())
    {
      delete *i;
      i++;
    }
  delete rules;
}

proposed_action_list * rules_behavior::do_actions(predicate_list * predicates, rules_line_type& rule_lines)
{
  //FILE * output;
  //output = fopen("./log/action.log", "a+");
  //fflush(output);
  //fprintf(output, "\n********************========================> DO_ACTION. Invocato metodo sul behavior: %s \n\n", this->get_name());
  //fflush(output);
  // It scrolls each rule to retrieve the actions
  rules_list::iterator i=rules->begin();
  // It creates the object to be returned
  proposed_action_list * pal = new proposed_action_list();
  while (i!=rules->end())
    {
      // temporaly variable to store the actions proposed by each rule
	   proposed_action_list * p=(*i)->get_actions(predicates);
      
	   // If there is no action p is NULL
      if (p!=NULL)
	{
	  // It puts every proposed action in the returned object
	  proposed_action_list::iterator y=p->begin();

	  char* behav_name = (char*)((*y).second)->get_behavior_name();
	  pair<int,float> line;
	  line.first = (*i)->GetLineNum();
	  line.second = ((*y).second)->get_membership_value();
	      
	  rule_lines[behav_name].push_back(line);

	  while (y!=p->end())
	    {

	      //fprintf(output, "***** DO_ACTION behavior '%s', alla riga %d, cerca di aggiungere:  %s <=> %s [ %f ]\n", behav_name, (*i)->GetLineNum(), ((*y).second)->get_name(), ((*y).second)->get_label(), ((*y).second)->get_membership_value());
	      //fflush(output);
	      pal->add((*y).second);
	      y++;
	    }
	}
      i++;
    }
  //fclose(output);
  return pal;
}

proposed_action_list * rules_behavior::filter_actions(predicate_list * predicates, proposed_action_list * aproplist, weight_want_list * want, composer * comp, fuzzyfier * fufy, preacher * priest, defuzzyfier * defufy, rules_line_type& rule_lines) {
  return NULL; // it does nothing since belongs to a level 1 behavior.
}


// class behavior_filter

behavior_filter::behavior_filter(char * name,rules_file_parser * arules_file_parser, int level)
  :behavior(name,level)

{
  rules_parser=arules_file_parser;
  rules=rules_parser->read_rules();
  // sets in each proposed action the behavior name attribute
  rules_list::iterator i=rules->begin();
  while (i!=rules->end())
    {
      proposed_action_list * pal=(*i)->get_actionslist();
      proposed_action_list::iterator y=pal->begin();
      while (y!=pal->end())
	{
	  (*y).second->set_behavior_name(get_name());
	  y++;
	}
      i++;
    }
}


behavior_filter::~behavior_filter()
{
  delete rules_parser;
  rules_list::iterator i=rules->begin();
  while (i!=rules->end())
    {
      delete *i;
      i++;
    }
  delete rules;
}


proposed_action_list * behavior_filter::do_actions(predicate_list * predicates, rules_line_type& rule_lines)
{
  return NULL;  // it does nothing since belongs to a level 2 (or more) behavior.
}


proposed_action_list * behavior_filter::filter_actions(predicate_list * predicates, proposed_action_list * aproplist, weight_want_list * want, composer * comp, fuzzyfier * fufy, preacher * priest, defuzzyfier * defufy, rules_line_type& rule_lines)
{

  FILE * output;
  output = fopen("./log/action.log", "a+");
  fflush(output);
  fprintf(output, "\n********************========================>FILTER_ACTION invocato metodo su behavior: %s \n\n", this->get_name());
  fflush(output);
  predicate_list * translated_predicates = 0;
  predicate_list * temporary_predicates = new predicate_list();
  char * mod;


  action_list * al = comp->compose(want,aproplist);

  command_list * cl =defufy->defuzzyfy(al);
  
  crisp_data_list * cdl = new crisp_data_list();

  command_list::iterator cl_it = cl->begin();
  while (cl_it != cl->end())
    {
      char * tmp = (char *)malloc(9 + strlen(cl_it->first));
      strcpy(tmp, "Proposed");
      strcat(tmp, cl_it->first);

      crisp_data_list::iterator i=cdl->find(tmp);
      if (i!=cdl->end())
	{
	  (*i).second->set_value(cl_it->second->get_set_point());
	  (*i).second->set_reliability(1.0);
	}
      else
	{
	  //printf("BRIAN: %s %f\n",cl_it->first,cl_it->second->get_set_point());
	  cdl->add(new crisp_data(tmp,cl_it->second->get_set_point(),1.0));
	}
      free(tmp);

      cl_it++;
    }

  fuzzy_data_list * fdl = fufy->fuzzyfy(cdl);

  
  predicate_list * pdl = priest->preach(fdl);

  //  predicates->delete_proposed();

  predicate_list::iterator p_it = pdl->begin();
  while (p_it != pdl->end())
    {
      predicates->add(p_it->second);
      p_it++;
    }

  for_each(cdl->begin(),cdl->end(),destroy_object<crisp_data>());
  delete cdl;
  for_each(fdl->begin(),fdl->end(),destroy_object<fuzzy_data>());
  delete fdl;
  for_each(al->begin(),al->end(),destroy_object<action>());
  delete al;
  for_each(cl->begin(),cl->end(),destroy_object<command>());
  delete cl;
  delete pdl;


  translated_predicates = translate(aproplist);
  *temporary_predicates = *predicates;
  if (translated_predicates != NULL) {
    predicate_list::iterator j=translated_predicates->begin();

    while (j != translated_predicates->end())
      {
	temporary_predicates->add((*j).second);
	j++;
      }
  }


  // It scrolls each rule to retrieve the actions
  rules_list::iterator i=rules->begin();
  while (i!=rules->end())
    {
      // temporaly variable to store the actions proposed by each rule
      proposed_action_list * p=(*i)->get_actions(temporary_predicates);
      // If there is no action p is NULL
      if (p!=NULL)
	{
	  // It puts every proposed action in the returned object
	  proposed_action_list::iterator y=p->begin();

	  char * behavior = (char*)this->get_name();

	  pair<int,float> line;
	  line.first = (*i)->GetLineNum();
	  line.second = ((*y).second)->get_membership_value();
	  
	  rule_lines[behavior].push_back(line);

	  while (y!=p->end())
            {
	      mod = ((*y).second)->get_modality();
	      if (strcmp(mod, "DEL") == 0) {
		const char * name = ((*y).second)->get_name();
		const char * label = ((*y).second)->get_label();


		#ifdef DEBUG
		printf("\n *** FILTER_ACTION behavior '%s', alla riga %d, cerca di CANCELLARE: %s <=> %s \n", behavior, (*i)->GetLineNum(), name, label);
		#endif
		fprintf(output, "*** FILTER_ACTION behavior '%s', alla riga %d, cerca di CANCELLARE: %s <=> %s \n", behavior, (*i)->GetLineNum(), name, label);
		//Added a parameter by mr:270603
		aproplist->del(name, label, behavior);
	      }
	      else if (strcmp(mod, "ADD") == 0) {
		#ifdef DEBUG
			printf("\n*** FILTER_ACTION behavior '%s', alla riga %d, cerca di aggiungere,: %s <=> %s [ %f]\n", this->get_name(), (*i)->GetLineNum(), ((*y).second)->get_name(), ((*y).second)->get_label(), ((*y).second)->get_membership_value());
		#endif
			char* action_name = (char*)((*y).second)->get_name();
		fprintf(output, "*** FILTER_ACTION behavior '%s', alla riga %d, cerca di aggiungere: %s <=> %s [ %f]\n", this->get_name(), (*i)->GetLineNum(), action_name, ((*y).second)->get_label(), ((*y).second)->get_membership_value());
		free (action_name);
		aproplist->add(new proposed_action (*((proposed_action *)(*y).second)));
	      }
	      else {
		fprintf(output, "**************FILTER_ACTION: ritornata condizione di ERRORE \n");
	      }
	      y++;
	      fflush(output);
	      free(mod);
            }
	}
      i++;
    }
  fclose(output);

  for (predicate_list::iterator it = translated_predicates->begin();
       it != translated_predicates->end(); ++it)
    {
      delete ((*it).second);
    }
  translated_predicates->clear();
  delete translated_predicates;

  temporary_predicates->clear();
  delete temporary_predicates;

  return aproplist;

}

char * behavior_filter::link_together (char * abehavior, char * aname, char * alabel) {
  // create a string like: &behavior.name.label

  char * tmp = (char *)malloc(strlen(abehavior) + strlen(aname) + strlen(alabel) + 4);
  strcpy(tmp, "&" );
  strcat(tmp, abehavior);
  strcat(tmp, ".");
  strcat(tmp, aname);
  strcat(tmp, ".");
  strcat(tmp, alabel);
  return tmp;
}


predicate_list * behavior_filter::translate (proposed_action_list * aproplist)
{
  char * name;
  char * label;
  char * behavior;
  float value;
  char * name_predicate;
  char * general_predicate;
  char * general_action;
  char * general_predicate_action;
  typedef map<string,float> myset;
  myset tmp_list;

  predicate_list * list = new predicate_list();
  proposed_action_list::iterator i = aproplist->begin();

  while (i != aproplist->end())
    {
      name = (char *)((*i).second->get_name());
      label = (char *)((*i).second->get_label());
      behavior = (char *)((*i).second->get_behavior_name());
      value = (*i).second->get_membership_value();
      name_predicate = link_together(behavior, name, label);
      list->add(new predicate((const char *)name_predicate, value, 1));
      free(name_predicate);
      general_predicate = link_together((char*)"SOMEBEHAVIOR", name, label);
      general_action = link_together(behavior,name,(char*)"ANY");
      general_predicate_action = link_together((char*)"SOMEBEHAVIOR",name,(char*)"ANY");
      tmp_list[general_predicate] = (*i).second->get_membership_value();
      tmp_list[general_action] = (*i).second->get_membership_value();
      tmp_list[general_predicate_action] = (*i).second->get_membership_value();
      free(name);
      free(general_predicate);
      free(general_action);
      free(general_predicate_action);
      i++;
    }

  myset::iterator it = tmp_list.begin();
  while (it != tmp_list.end())
    {
      //converto un tipo string in un char*
      list->add(new predicate((*it).first.c_str(), (*it).second, 1));
      it++;
    }

  return (list);
}
