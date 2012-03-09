/***************************************************************************
                          wanter_eng.cpp  -  description
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

#include "wanter_eng.h"

// Abstract Class Wanter

wanter::wanter()
{
}

wanter::~wanter()
{
}

// Class Predicate Wanter

predicate_wanter::predicate_wanter(char * afilename)
{
  aggr_tree_parser * wcdp = new aggr_tree_parser(afilename);
  wanttree = new cando_tree_map();
  wcdp->get_aggr_trees(wanttree);
  delete wcdp;
}

predicate_wanter::~predicate_wanter()
{
  delete wanttree;
}

weight_want_list * predicate_wanter::what_do_you_want (predicate_list * predicates)
{
  weight_want_list * wwl = new weight_want_list();
  predicate_list * wantlist = wanttree->evalue(predicates);//elimina i want certamente falsi...(mf=0 rel=1)
  predicate_list::iterator i = wantlist->begin();

  while ( i != wantlist->end() )
    {
  	   const char *name=(*i).second->get_name();
      wwl->add(new weight_want(name,(*i).second->get_value()));
      i++;
    }

  wantlist->clear();
  delete wantlist;
  return wwl;
}

//what_do_you_want con filtro, a seconda del filtro passato, la lista viene epurata in base
//a value, reliability o entrambi
weight_want_list * predicate_wanter::what_do_you_want (predicate_list * predicates, can_do_filter *acandofilter)
{
	weight_want_list *wwl=new weight_want_list();
	predicate_list *wantlist= wanttree->evalue(predicates);
	predicate_list *filtWantlist;
	can_do_filter *flt=acandofilter;
		
	//prima filtro i want...
	filtWantlist=flt->filter(wantlist);

	//...poi li aggiungo alla lista dei pesi
	predicate_list::iterator i=filtWantlist->begin();
	while (i!=filtWantlist->end())
		{
		const char *name=(*i).second->get_name();
		wwl->add(new weight_want(name,(*i).second->get_value()));
		i++;
		}

	for_each(filtWantlist->begin(),filtWantlist->end(),destroy_object<predicate>());	
	filtWantlist->clear();
	delete filtWantlist;

	for_each(wantlist->begin(),wantlist->end(),destroy_object<predicate>());	
	wantlist->clear();
	delete wantlist;
	return wwl;
}
