/***************************************************************************
                          behavior_eng.cpp  -  description
                             -------------------
    begin                : Sat Sep 30 2000
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

#include "behavior_eng.h"
#include "rules_behav.h"
#include <iostream>

#include "smdebug.h"

// Class behavior_engine

//costruttore di default
behavior_engine::behavior_engine()
      : behlist(NULL), candofilter(NULL), behparser(NULL)
{
}



//costruttore per copia 
behavior_engine::behavior_engine(const behavior_engine &right)
      : behlist(NULL), candofilter(NULL)
{
  this->behlist=right.behlist;
  this->candofilter=right.candofilter;
  this->behparser=right.behparser;
}



//costruttore
behavior_engine::behavior_engine (behavior_parser * abehaviorparser, can_do_filter * acandofilter)
{
  candofilter=acandofilter;
  behparser=abehaviorparser;
  behlist=behparser->read_behavior();
}

behavior_engine::~behavior_engine()
{
  delete candofilter;
  delete behparser;
  delete behlist;
}


//## Other Operations (implementation)

proposed_action_list * behavior_engine::run_engine (predicate_list * predicates, predicate_list * cando, weight_want_list * want, composer * comp, fuzzyfier * fufy, preacher * priest, defuzzyfier * defufy, rules_line_type& rule_lines)
{
  //FILE * output;
  predicate_list * active;
  
  proposed_action_list * list = new proposed_action_list();
  //filtro i cando (per mf, per reliability o per entrambi a seconda del filtro)
  active=candofilter->filter(cando);
  
  //output = fopen("./log/action.log", "a+"); //un file di log delle operazioni eseguite dall'engine
  predicate_list::iterator i = active->begin();
 
  //printf("\nIN run_engine(): start while (i!=active->end())");
  
  while ( i!=active->end())
    {
		const char * name = (*i).second->get_name();//prende il nome del primo predicato dalla lista di cando
      
      behavior * curr = behlist->get_behavior(name);//dalla lista comportamenti prende il comportamento di nome "name"
      
      proposed_action_list * pal = curr->do_actions(predicates,rule_lines);//estrae le azioni proposte dal particolare comportamento

#ifdef DEBUG
      printf("\nIN while(...):");
      printf("\nBehavior name: %s ->do_action()",name);
      printf("\nPal proposed by behavior %s:",name);
      if (pal==NULL) printf("	is empty");
      else viewpacl(pal);
      printf("\n--------------------------------------------------------------------");
#endif
	
	 


      if (pal != NULL) 
	{//se ci sono azioni proposte...
	  proposed_action_list::iterator j = pal->begin();
	  while (j != pal->end())
	    {
	      list->add(new proposed_action (*((proposed_action *)(*j).second)));//...le aggiunge tutte alla lista da ritornare
	      j++;
	    }
	  for_each(pal->begin(),pal->end(),destroy_object_first<proposed_action>());
	}
      i++;//...lo fa per ognuno dei comportamenti
      
    }
  
  int n = 2;//parte dal livello 2
  bool exit = false;
  while (!exit) {
    can_do_list * curr_list = behlist->get_behavior_filter(n);//prende tutti i comportamenti di livello n...
    if (curr_list->size() > 0) 
      {//se ci sono comportamenti di livello n...
	
	can_do_list::iterator k = curr_list->begin();
	
	while (k != curr_list->end())//...per tutti i comportamenti di livello n
	  {
	    if (active->find((*k).first) != active->end())//...scorre tutta la lista di cando (filtrata)
	      {
		behavior * curr = (*k).second;
		//fflush(output);
		//fprintf(output, "\n=================>  RUN_ENGINE 2° ciclo. Nome behavior: %s. Livello = %i \n", curr->get_name(), n);
		//fflush(output);
		list = curr->filter_actions(predicates, list,want, comp,fufy,priest,defufy,rule_lines);//lista di azioni proposte e filtrate
#ifdef DEBUG
		printf("\nIn WHILE (with n>2) = %d",n);
		printf("\nBehavior name: %s ->filter_actions()",curr->get_name());
		printf("\nPal proposed after filter_actions():");
		viewpacl(list);
#endif
	      }
	    k++;//...passa al successivo comportamento di livello n
	  }
      }
    else  {
      exit = true;
    }
    curr_list->clear();
    delete curr_list;
    n++;//passa al comportamento di livello n+1
  }
  //fflush(output);
  //fprintf(output, "\n*********************RUN_ENGINE. Lista di azioni da ritornare*******************\n\n");
  //fflush(output);
  
  for_each(active->begin(),active->end(),destroy_object<predicate>());	
  active->clear();
  delete active;
  
  proposed_action_list::iterator j = list->begin();
  
  while (j != list->end())
    {
      //fflush(output);
      char* action_name = (char*)((*j).second)->get_name();
      //fprintf(output, "*** RUN_ENGINE proposed_action ritornata : %s <=> %s proposta da: %s \n", action_name, ((*j).second)->get_label(), ((*j).second)->get_behavior_name());
      free(action_name);
      //fflush(output);
      j++;
    }
  //fflush(output);
  //fprintf(output, "\n*********************RUN_ENGINE. FINE lista di azioni da ritornare*******************\n\n");
  //fflush(output);
  //fclose(output);
  return(list);
  
}


// Class can_do_filter

can_do_filter::can_do_filter()
{
}

can_do_filter::can_do_filter(const can_do_filter &right)
{
}


can_do_filter::~can_do_filter()
{
}

// Class behavior_parser

behavior_parser::behavior_parser()
  :filename(NULL)
{
}

behavior_parser::behavior_parser(const behavior_parser &right)
  :filename(NULL)
{
  filename=(char *) malloc(strlen(right.filename)+1);
  strcpy(this->filename,right.filename);
}

behavior_parser::behavior_parser (char * afilename)
{
  filename=(char *) malloc(strlen(afilename)+1);
  strcpy(filename,afilename);
}


behavior_parser::~behavior_parser()
{
  if (filename!=NULL) free(filename);
}

//## Other Operations (implementation)

can_do_list * behavior_parser::read_behavior ()
   {
	   
	  cout<<"Read_Bahaviour";
	  	   
      FILE * input;
      //FILE * output;
      char * behname;
      char * file;
      int level;

      behname=(char *)malloc(80);
      file=(char *)malloc(80);

	  can_do_list * list = new can_do_list();

	  input = fopen(filename,"r");
      //output = fopen("/home/cristian/log/action.log", "w+");

      cout << '\n';
      while (!feof(input))
      {
         fscanf( input, "( level %i %s %s )\n", &level, behname, file);
         
         //cout<<"Control 0";


         rules_file_parser * rfp = new rules_file_parser(file);
         
         //cout<<"Control 1";
         
         //fflush(output);
         
         //cout<<"Control 2";
       
         //fprintf(output, "individuato behavior di livello %i : %s \n", level, behname);
         
         //fflush(output);
         if (level == 1) 
         {
	        rules_behavior * curr = new rules_behavior(behname,rfp);
            list->add_behavior(curr);
            //cout<<"Control 3";

         }
         
         else if (level > 1) 
         {
	         behavior_filter * curr = new behavior_filter(behname, rfp, level);
            list->add_behavior(curr);
         }
         else {
            //fflush(output);
            //fprintf(output, "Behavior_parser: PLEASE, INSERT A VALID LEVEL \n");
            //fflush(output);

         }
      }
      fclose (input);
      //fclose (output);
      free (behname);
      free (file);
      return (list);
   }




// Class threshold_filter
threshold_filter::threshold_filter()
      : threshold(0.49)
{
}

threshold_filter::threshold_filter(const threshold_filter &right)
      : threshold()
{
  threshold=right.threshold;
}

threshold_filter::~threshold_filter()
{
}

// Additional Declarations

//Filtra i predicati in base al threshold (sul valore)
predicate_list * threshold_filter::filter (predicate_list * candolist)
{
  predicate_list * list = new predicate_list();
  predicate_list::iterator i;

  i=candolist->begin();
  while (i!=candolist->end())
    {
      if (((*i).second)->get_value()>threshold) //filtro i predicati, prendo quelli dove value>threshold
			list->add(new predicate(*((*i).second) ) );
      i++;
    }
  return (list);
}




//Saverio Morpurgo: Class rthreshold_filter
rthreshold_filter::rthreshold_filter()
      : rthreshold(0.49)
{
}
rthreshold_filter::rthreshold_filter(const rthreshold_filter &right)
      : rthreshold()
{
  rthreshold=right.rthreshold;
}




rthreshold_filter::~rthreshold_filter()
{
}


//Filtra i predicati in base al threshold (sulla reliability)
predicate_list * rthreshold_filter::filter(predicate_list * candolist)
{
  predicate_list * list = new predicate_list(); //lista da ritornare (filtrata)
  predicate_list::iterator i;

  i=candolist->begin();
  while (i!=candolist->end())
    {
      if (((*i).second)->get_reliability()>rthreshold) //filtro i predicati, prendo quelli dove reliability>threshold
			list->add(new predicate(*((*i).second) ) );
      i++;
    }
  return (list);
}
//END RThreshold filter*/

//Saverio Morpurgo: CLASS COMPOSITE FILTER, estende cando_filter e filtra per entrambi i valori di threshold (value e reliability)
composite_filter::composite_filter():threshold(0.49),rthreshold(0.49)
{
}

composite_filter::composite_filter(const composite_filter &right)
		:threshold()
{
threshold=right.threshold;
rthreshold=right.rthreshold;
}

composite_filter::~composite_filter()
{
}

predicate_list * composite_filter::filter (predicate_list *candolist)
{
  	predicate_list *list=new predicate_list(); //lista da ritornare (filtrata)
	predicate_list::iterator i;

   i=candolist->begin();
	while (i!=candolist->end())
	  {
	  //qui filtro rispetto ad entrambi i threshold, sul valore e sulla reliability
	  if (( ((*i).second)->get_value()>threshold )&&(((*i).second)->get_reliability()>rthreshold )) 
        list->add(new predicate(*((*i).second)));
	  i++;
	  }
  return list;
}
//END COMPOSITE FILTER


// Class can_do_list

can_do_list::can_do_list()
  :can_do_multimap()
{
}

can_do_list::can_do_list(const can_do_list &right)
  :can_do_multimap(right)
{
}

can_do_list::~can_do_list()
{
  iterator i=begin();
  while (i!=end())
    {
      delete (*i).second;
      i++;
    }
}

behavior * can_do_list::get_behavior (const char * aname)
{
  iterator i;

  i=find(aname);
  if (i==end())
    return NULL;
  else
    return (behavior *) (*i).second;

}


void can_do_list::add_behavior (behavior * abeh)
{
  insert(pair<const char *, behavior *>(abeh->get_name(),abeh));
}


can_do_list * can_do_list::get_behavior_filter(int alevel)

   {
      int level;
      can_do_list * behlist = new can_do_list();
      iterator i = begin();

      while (i != end()) {

         level = (*i).second->get_level();

         if (level == alevel) {
            behlist->add_behavior((*i).second);
         }

         i++;
      }

      return behlist;
   }



// Class behavior


behavior::behavior()
{
  name=NULL;
}



behavior::behavior(const behavior &right)
   {
      name=(char *) malloc(strlen(right.name)+1);
      strcpy(this->name,right.name);
      this->level = right.level;
   }

behavior::behavior(char * aname)
   {
      name=(char *) malloc(strlen(aname)+1);
      strcpy(name,aname);
      level = 1;
   }

behavior::behavior(char * aname, int alevel)
   {
      name = (char *) malloc(strlen(aname)+1);
      strcpy(name, aname);
      level = alevel;
   }



behavior::~behavior()
{
  if (name!=NULL) free(name);
}
