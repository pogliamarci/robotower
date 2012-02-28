/***************************************************************************
                          brian.cpp  -  description
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

#include "brian.h"

#include "smdebug.h"

int getTime()
{
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv,&tz);
  return(tv.tv_sec * 1000000 + tv.tv_usec);
}

MrBrian::MrBrian(char * fuzzyassoc, 
	         char * fuzzyshapes, 
		 char * pries, 
		 char * actionpries, 
		 char * candoes, 
		 char * behaviors, 
		 char * wanters, 
		 char * defuzzyassoc, 
		 char * defuzzyshapes)		 
{
  #ifdef DEBUG
  	printf("new MrBrian: new GetFuzzy()\n");
  #endif
  
  fuzzy = new GetFuzzy(fuzzyassoc,
		       fuzzyshapes,
		       defuzzyassoc,
		       defuzzyshapes);
  
  #ifdef DEBUG
  	printf("new MrBrian: new preacher(pries)\n");
  #endif
  
  // preacher
  priest=new preacher(new aggr_tree_parser(pries));

  #ifdef DEBUG
  	printf("new MrBrian: new preacher(actionpries)\n");
  #endif
  	
  // preacher
  actionpriest=new preacher(new aggr_tree_parser(actionpries));

  
  #ifdef DEBUG
  	printf("new MrBrian: new candoer()\n");
  #endif
  
  // candoer
  allower=new candoer(new aggr_tree_parser(candoes));


  //crea filtro con costrutto di default, composito (membership e reliability)
  composite_filter *filter = new composite_filter();
  
  //crea filtro con costruttore di default (default threshold = 0.4 ) per reliability
  //rthreshold_filter *filter = new rthreshold_filter();
  
  //crea filtro con costruttore di default (default threshold = 0.4 ) per il valore
  //threshold_filter * f = new threshold_filter();
  
  
  #ifdef DEBUG
  	printf("new MrBrian: new behavior_parser()\n");
  #endif
  
  //crea beahvior parser sul file beahvior
  behavior_parser * bp = new behavior_parser(behaviors);
  
  
  #ifdef DEBUG
  	printf("new MrBrian: new behavior_engine()\n");
  #endif
  
  // behavior engine
  beg = new behavior_engine(bp,filter);

  #ifdef DEBUG
  	printf("new MrBrian: new wanter()\n");
  #endif
  
  // wanter
  want = new predicate_wanter(wanters);
  
  // composer
  weight_composer * wc = new mult_weight_composer();
  float_composer *fc = new max_float_composer();
  comp = new composer(wc,fc);

  pdl=new predicate_list();
  cadl=new predicate_list();
  pal=new proposed_action_list();
  wwl=new weight_want_list();
}

MrBrian::MrBrian()
{
  priest = new preacher();
  allower = new candoer();
  beg = new behavior_engine();
  comp = new composer();

  pdl=new predicate_list();
  cadl=new predicate_list();
  pal=new proposed_action_list();
  wwl=new weight_want_list();
}

MrBrian::~MrBrian()
{
  delete fuzzy;
  delete priest;
  delete actionpriest;
  delete allower;
  delete beg;
  delete want;
  delete comp;

  delete pdl;
  delete cadl;
  delete pal;
  delete wwl;
}


void MrBrian::run()
{

  // DO YOUR DUTY
  time = getTime();


  // fuzzyfication
  fuzzy->set_fuzzy_data_list((fuzzy->get_fuzzyfier())->fuzzyfy(fuzzy->get_crisp_data_list()));


  // predicate list
  pdl = priest->preach(fuzzy->get_fuzzy_data_list());
  

  // can_do list
  cadl = allower->can_i(pdl);

  // weight want list
  can_do_filter *cf=new composite_filter();//filtro dei cando (composito)
  wwl = want->what_do_you_want(pdl,cf);
  delete cf;

  //wwl = want->what_do_you_want(pdl); //vecchia chiamata, senza filtro

  // behavior engine
  //**********************  DA QUI ***********************************************
  #ifdef DEBUG
  	printf("\npal=beg->(pdl,cadl,wwl,comp,fuzzy->get_fuzzyfier(), actionpriest, fuzzy->get_singleton());");
  #endif
  rules_lines.clear();
  pal = beg->run_engine(pdl,cadl,wwl,comp,fuzzy->get_fuzzyfier(), actionpriest, fuzzy->get_singleton(),rules_lines);
 
  // composer
  fuzzy->set_action_list(comp->compose(wwl,pal));
  
  // defuzzyfication
  fuzzy->set_command_singleton_list((fuzzy->get_singleton())->defuzzyfy(fuzzy->get_action_list()));

  time = getTime()-time;
  //printf("\nTIME: %d\n",time);
}


int MrBrian::get_time()
{
  return time;
}

void MrBrian::flush()
{
  fuzzy->flush();
  for_each(pdl->begin(),pdl->end(),destroy_object<predicate>());
  for_each(cadl->begin(),cadl->end(),destroy_object<predicate>());
  for_each(pal->begin(),pal->end(),destroy_object_complete<proposed_action>());
  for_each(wwl->begin(),wwl->end(),destroy_object<weight_want>());

  pdl->clear();
  cadl->clear();
  pal->clear();
  wwl->clear(); 

  delete pdl;
  delete cadl;
  delete pal;
  delete wwl;

  //  pdl=new predicate_list();
  //  cadl=new predicate_list();
  //  pal=new proposed_action_list();
  //  wwl=new weight_want_list();
}

void MrBrian::debug()
{
  fuzzy->debug();
  viewpbpl(pdl);
  viewwwl(wwl);
  viewpacl(pal);
}

weight_want_list * MrBrian::get_weight_want_list()
{
  return wwl;
}

predicate_list * MrBrian::get_predicate_list()
{
  return pdl;
}

predicate_list * MrBrian::get_cando_list()
{
  return cadl;
}

proposed_action_list * MrBrian::get_proposed_action_list()
{
  return pal;
}

rules_line_type* MrBrian::get_rules_lines()
{
  return &rules_lines;
}

GetFuzzy * MrBrian::getFuzzy()
{
  return fuzzy;
}
