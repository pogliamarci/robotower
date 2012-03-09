/***************************************************************************
                          can_doer.cpp  -  description
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

//	Description: This module at strart up create the
//	Predicate tree and its list by the predicate parser. 
//	Durind the runtime  evalue the predicate tree by the 
//	fuzzy data list. 
//
//------------------------------------------------------------


/* ======================== ATTENZIONE !!!! =======================

   I file di configurazione devono terminare con una linea scritta
   secondo la grammatica e non con un newline
   NON ADATE A CAPO DOPO L'ULTIMA RIGA!!!!!

   ================================================================ */


#include "can_doer.h"

#include "smdebug.h"

cando_tree_map::cando_tree_map()
  :aggr_tree_multimap()
{
}

cando_tree_map::cando_tree_map(const cando_tree_map &right)
  :aggr_tree_multimap(right)
{
}

cando_tree_map::~cando_tree_map()
{
  for_each(begin(),end(),destroy_object<aggregation_tree> ());
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

predicate_list * cando_tree_map::evalue (predicate_list * prd_lst)
{
   	predicate_list * cnd_lst = NULL;
   	iterator i;

  	// Creo la lista di predicati da ritornare
  	cnd_lst = new predicate_list();

	fuzzy_data_list * fdl = new fuzzy_data_list();

  	for (i = begin(); i != end(); i++) {

	  // Prendo il puntatore all'albero delle cando.
	  aggregation_tree *cnd_tree = (*i).second;
	  
	  char* name = cnd_tree->get_name(); 
	  float value = cnd_tree->get_value(prd_lst, fdl);
	  float reliability = cnd_tree->get_reliability(prd_lst, fdl);
	  
	  //modified in version 2.1
	  if (value != 0 || reliability != 1) 
	    {
	      predicate * current = new predicate (name,value,reliability);

	      cnd_lst->add(current);
	    }

	  delete[] name;
	  //===================================================================
	  // Per la visualizzazione del contenuto della lista
	  #ifdef DEBUG
	  printf ("\ncando_tree_map->evalue() : %s\nValue : %f\nReliability :%f ",
	  cnd_tree->get_name(), cnd_tree->get_value(prd_lst,fdl), cnd_tree->get_reliability(prd_lst,fdl));
	  printf("\nLa lista di predicati viene epurata solo dei predicati certamente falsi (value 0, reliability 1 )");    
	  #endif
  	  //===================================================================
	  //===================================================================

	};// End for

	delete fdl;

	return cnd_lst;
	// end prd_tree_map::evalue%
}
//-------------------------------------------------------------------------

candoer::candoer()
{
  cando_parser=NULL;
  cando=NULL;
}

candoer::candoer(aggr_tree_parser * acandoparser)
{
  cando_parser=acandoparser;
  cando = new cando_tree_map();
  cando_parser->get_aggr_trees(cando);
}

candoer::~candoer()
{
  delete cando_parser;
  delete cando;
}

predicate_list * candoer::can_i(predicate_list * predicates)
{
  return cando->evalue(predicates);
}
