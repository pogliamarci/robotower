/***************************************************************************
                          behavior_eng.h  -  description
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

#ifndef behavior_eng_h
#define behavior_eng_h 1

#include <cstring>

#include "getFuzzy.h"
#include "preacher.h"
#include "can_doer.h"
#include "compose.h"

#ifdef DMALLOC
#include <dmalloc.h>
#endif

typedef map<char*,vector<pair<float,float> > > rules_line_type;

/**
 * Abstract class. It is used to filter cando values in order to choose behaviors
 * to run.
 * @short General cando filter
 * @see behavior_engine
 * @see candoer
 */
class can_do_filter
{
  public:
      /**
       * Constructor.
       * @return A filter for cando values.
       */
      can_do_filter();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @return A copy of the given object
       */
      can_do_filter(const can_do_filter &right);

    //## Destructor (generated)
      /**
       * Destructor
       */
      virtual ~can_do_filter();

      /**
       * It choose which behavior must be run.
       * Behavior names are stored as a name of a pred_beh_parent object.
       * @param candolist List of cando values, i.e. a couple made of behavior name and his cando computed value.
       * @return A list of behavior to be activated.
       * @see pred_beh_parent
       */
      virtual predicate_list * filter (predicate_list * candolist)=0;
};

//	Class that implements the threshold behavior filter.
/**
 * A threshold filter for cando values. It gives back only behaviors whose cando value
 * is higher than a fixed threshold.
 * Defalut threshold value is 0.49.
 * @short Threshold filter for cando values.
 * @see behavior_engine
 * @see candoer
 */
class threshold_filter : public can_do_filter
{
  public:
    //## Constructors (generated)
      /**
       * Constructor. It set threshold value to 0.49.
       * @return A threshold filter.
       */
      threshold_filter();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @return A copy of the given object
       */
      threshold_filter(const threshold_filter &right);

      /**
       * Destructor
       */
      virtual ~threshold_filter();

      /**
       * Method to read read threshold value.
       * @return Threshold value.
       */
      float get_threshold ();
      
      /**
       * Method to set threshold value.
       * @param value New threshold value.
       */
      void set_threshold (float value);

      /**
       * It scrolls incoming list to find wich behaviors have activation values
       * greater than the threshold, and copies them into tha outgoing list.
       * @param candolist List of cando values to look into.
       * @return Behaviors whose cando valuee are bigger than the threshold.
       */
      virtual predicate_list * filter (predicate_list * candolist);

  private:
      float threshold;
};

//Saverio Morpurgo: Class rthreshold_filter (filtra i predicati in base alla reliability)
class rthreshold_filter : public can_do_filter
{
  public:
    //## Constructors (generated)
      /**
       * Constructor. It set threshold value to 0.49.
       * @return A threshold filter.
       */
      rthreshold_filter();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @return A copy of the given object
       */
      rthreshold_filter(const rthreshold_filter &right);

      /**
       * Destructor
       */
      virtual ~rthreshold_filter();

      /**
       * Method to read read threshold value.
       * @return Threshold value.
       */
      float get_rthreshold ();
      
      /**
       * Method to set threshold value.
       * @param value New threshold value.
       */
      void set_rthreshold (float reliability);

      /**
       * It scrolls incoming list to find wich behaviors have activation values
       * greater than the threshold, and copies them into tha outgoing list.
       * @param candolist List of cando values to look into.
       * @return Behaviors whose cando valuee are bigger than the threshold.
       */
      virtual predicate_list * filter (predicate_list * candolist);

  private:
      float rthreshold;
};
//END RTHreshold filter 

class composite_filter : public can_do_filter
{
  public:
    //## Constructors (generated)
      /**
       * Constructor. It set threshold value to 0.49.
       * @return A threshold filter.
       */
      composite_filter();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @return A copy of the given object
       */
      composite_filter(const composite_filter &right);

      /**
       * Destructor
       */
      virtual ~composite_filter();

      /**
       * Method to read read threshold value.
       * @return Threshold value.
       */
      float get_threshold();
      float get_rthreshold();
      
      /**
       * Method to set threshold value.
       * @param value New threshold value.
       */
      void set_threshold (float value);
		void set_rthreshold (float reliability);
		void set_composite_threshold(float value, float reliability);

      /**
       * It scrolls incoming list to find wich behaviors have activation values
       * greater than the threshold, and copies them into tha outgoing list.
       * @param candolist List of cando values to look into.
       * @return Behaviors whose cando valuee are bigger than the threshold.
       */
      virtual predicate_list * filter (predicate_list * candolist);

  private:
      float threshold;
      float rthreshold;
};

// Class behavior
/**
 * This class is the description of a general behavior, made of a name and a method to be called by the engine.
 * Each new kind of behvior you want to add must inherit from it to be able to be used by behavior_engine.
 * @short Abstract behavior
 * @see behavior_engine
 */
class behavior
{
  public:
      /**
       * Constructor.
       * @return An empty behavior.
       */
      behavior();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @return A copy of the given object
       */
      behavior(const behavior &right);

      /**
       * Constructor
       * @param name Name of the behavior
       * @return A behavior with given name
       */
      behavior(char * aname);


      behavior(char * aname, int alevel);

      /**
       * Destructor
       */
      virtual ~behavior();

      /**
       * Method to execute behavior. It receives a list of predicates and proposes some actions (or none at all)
       * @param apredlist Predicate list
       * @return List of proposed actions.
       */
      virtual proposed_action_list * do_actions (predicate_list * apredlist, rules_line_type& rule_lines)=0;

      /**
       * Method to execute behavior filter. It receives a list of predicates and of actions proposed
         by behaviors of a previous level.
       * @param apredlist Predicate list
       * @return List of proposed actions.
       */
      virtual proposed_action_list * filter_actions (predicate_list * predicates, proposed_action_list * aproplist, weight_want_list * want, composer * comp, fuzzyfier * fufy, preacher * priest, defuzzyfier * defufy, rules_line_type& rule_lines)=0;
   

    // Additional Public Declarations
      /**
       * Method to read behavior name.
       * @return pointer to behavior name.
       */
      const char * get_name();

      int get_level();

  private:
      char * name;
      int level;
};

/**
 * Instantiation of template multimap in order to create a behavior base
 * @see can_do_list
 */
typedef multimap<const char *,behavior *,ltstr > can_do_multimap;


// Class can_do_list
/**
 * List of behaviors, i.e. the behavior base.
 * @see behavior_engine
 * @see behavior
 */
class can_do_list : public can_do_multimap
{

 public:
  /**
   * Constructor. It simply calls its parent's constructor.
   * @return A behavior base.
   */
  can_do_list();

  /**
   * Copy constructor.
   * @param right Object to be copied.
   * @return A copy of the given object
   */
  can_do_list(const can_do_list &right);
  
  /**
   * Destructor. It recursively deletes all objects it points to.
   */
  virtual ~can_do_list();

  /**
   * It looks for a behavior with the specified name and gives back a pointer to it.
   * @param aname Name of the behavior to look for.
   * @result A pointer to the desired behavior if found, NULL otherwise.
   */
  virtual behavior * get_behavior (const char * aname);

  /**
   * Add a behavior to the list.
   * @param abeh Pointer to behavior to add.
   */
  virtual void add_behavior (behavior * abeh);

  /**
   * It looks for all behaviors with the specified level and gives back a pointer to a list of behaviors.
   * @param level of the behavior to look for.
   * @result a pointer to list of behavior found.
   */
      virtual can_do_list * get_behavior_filter(int level);

};

//	Class that read from a file and make the list of
//	behavior.
/**
 * It reads from file a list of behaviors to instantiate.
 * It was implemented to instantiate only rules behaviors.
 * @short Behavior list file reader.
 * @see behavior_engine
 * @see can_do_list
 * @see rules_behavior
 */
class behavior_parser
{
  public:
      /**
       * Constructor
       * @return An empty behavior parser.
       */
      behavior_parser();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @return A copy of the given object
       */
      behavior_parser(const behavior_parser &right);
      
      /**
       * Constructor. It reads behavior list from specified file.
       * @return A behavior parser which will read from given file.
       * @param afilename File to read from.
       */
      behavior_parser (char * afilename);

      /**
       * Destructor.
       */
      virtual ~behavior_parser();
      
      /**
       * It reads behavior name and optional parameters from file, and give back a list of instantiated behaviors.
       * @return The behavior base. If error, an empty list is returned.
       */
      virtual can_do_list * read_behavior ();

  private:
      char * filename;
};

//	class that implements the behavior engine.
/**
* This class handles behaviors activations and collects their results.
* @short Behavior engine
* @see behavior
* @see behavior_parser
* @see can_do_list
* @see can_do_filter
*/
class behavior_engine
{
  public:
      /**
       * Constructor.
       * @return An empty (that is: not working) behavior engine
       */
      behavior_engine();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @return A copy of the given object
       */
      behavior_engine(const behavior_engine &right);

       /**
       * Constructor. It is possible to specify objects the engine needs.
       * @return A behavior engine that will work with specified elements.
       * @param abehaviorparser Pointer to behavior parser to use to create behavior base
       * @param acandofilter Pointer to cando filter to use.
       */
      behavior_engine (behavior_parser * abehaviorparser, can_do_filter * acandofilter);
  
  	  /**
       * Destructor. It deletes behavior parser and cando filter too.
       */
      virtual ~behavior_engine();

      /**
       * Method to execute the engine. First it uses a cando filter to know wich
       * behavior must be executed, and then calls them collecting their results.
       * @return A list of proposed actions from activated behaviors. 
       * @param predicates List of predicates.
       * @param cando List of cando values.
       * @param want List of want values.
       * @param comp Composer.
       * @param fufy Fuzzyfier.
       * @param priest Preacher.
       * @param defufy Defuzzyfier.
       * @see can_do_filter
       */
      virtual proposed_action_list * run_engine (predicate_list * predicates, predicate_list * cando, weight_want_list * want, composer * comp, fuzzyfier * fufy, preacher * priest, defuzzyfier * defufy, rules_line_type& rule_lines);

  private: //## implementation
      can_do_list * behlist; //lista dei comportamenti estratta da dal parser
      can_do_filter * candofilter; //filtro
  		behavior_parser * behparser; //parser sul file dei comportamenti
};




//get e set threshold per threshold_filter
inline float threshold_filter::get_threshold ()
{
  return threshold;
}

inline void threshold_filter::set_threshold (float value)
{
  threshold = value;
}



//get e set rthreshold per rthreshold_filter
inline float rthreshold_filter::get_rthreshold ()
{
  return rthreshold;
}

inline void rthreshold_filter::set_rthreshold (float reliability)
{
  rthreshold = reliability;
}

//get e set per composite_filter 
inline float composite_filter::get_threshold()
{
	return threshold;
}

inline float composite_filter::get_rthreshold()
{
	return rthreshold;
}

inline void composite_filter::set_threshold(float value)
{
threshold=value;
}

inline void composite_filter::set_rthreshold(float reliability)
{
rthreshold=reliability;
}


inline void composite_filter::set_composite_threshold(float value, float reliability)
{
threshold=value;
rthreshold=reliability;
}


// Class behavior
inline const char * behavior::get_name()
{
  return name;
}

inline int behavior::get_level()
   {
      return level;
   }

#endif
