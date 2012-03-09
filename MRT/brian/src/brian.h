/***************************************************************************
                          Brian.h  -  description
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

#ifndef mr_brian_h
#define mr_brian_h 1

#include "interf_obj.h"
#include "preacher.h"
#include "can_doer.h"
#include "wanter_eng.h"
#include "behavior_eng.h"
#include "compose.h"
#include <sys/time.h>
#include <unistd.h>

#include <getFuzzy.h>

#ifdef DMALLOC
#include <dmalloc.h>
#endif


/**
 * Brian main class. It sets up all the elements it needs to work and calls them in the
 * sequence right.
 * @short Brian Main Class
 * @see fuzzyfier
 * @see preacher
 * @see cando_tree_map
 * @see behavior_engine
 * @see wanter
 * composer
 * defuzzyfier
 */
class MrBrian
{

private:

  GetFuzzy* fuzzy;
  
  preacher * priest;
  preacher * actionpriest;
  candoer * allower;
  behavior_engine * beg;
  wanter * want;
  composer * comp;
  
  predicate_list * pdl;
  predicate_list * cadl;
  proposed_action_list * pal;
  weight_want_list * wwl;

  rules_line_type rules_lines;

  int time;

 public:

  /**
   * Constructor. It uses configuration files to set up all Brian's modules.
   * @return BRIAN
   * @param fuzzyassoc File name in which associations betweed data and shapes are defined for fuzzyfication module.
   * @param fuzzyshapes File name in which shapes are defined for fuzzyfication module.
   * @param pries File name in which predicates are defined.
   * @param candoes File name in which cando definitions are defined.
   * @param behaviors File name in which behavior base is defined.
   * @param wanters File name in which want definitions are defined.
   * @param defuzzyassoc File name in which associations betweed data and shapes are defined for defuzzyfication module.
   * @param defuzzyshapes File name in which shapes are defined for fuzzyfication module. 
   */
  
  MrBrian(char * fuzzyassoc, 
	  char * fuzzyshapes, 
	  char * pries, 
	  char * actionpries, 
	  char * candoes, 
	  char * behaviors, 
	  char * wanters, 
	  char * defuzzyassoc, 
	  char * defuzzyshapes);

  MrBrian();

  /**
   * Destructor.
   */
  ~MrBrian();

  /**
   * Run Brian inferential cycle.
   */
  void run();

  int get_time();

  void flush();

  void debug();

  weight_want_list * get_weight_want_list();
  predicate_list * get_predicate_list();
  predicate_list * get_cando_list();
  proposed_action_list * get_proposed_action_list();
  rules_line_type* get_rules_lines();

  GetFuzzy * getFuzzy();

};

#endif
