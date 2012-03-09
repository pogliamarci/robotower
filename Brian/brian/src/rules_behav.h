/***************************************************************************
                          rules_behav.h  -  description
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

#include "behavior_eng.h"


#ifdef DMALLOC
#include <dmalloc.h>
#endif

// Class rule implements the "if ... then ..." condition of a rule in a rules behavior
/**
 * This class implements a rule, made of a precondition and a list of actions.
 * @short Rule in a rules behavior
 * @see rules_behavior
 * @see rules_file_parser
 * @see rules_list
 */
   class rule
   {
   public:
   
   // default constructor
   /**
   * Constructor.
   * @return An empty rule.
   */
      rule();
   
   /**
   * Constructor. Precondition and list of actions are set by parameters.
   * @return A rule with given precondition and action list
   * @param conds Pointer to a predicate tree defining a precondition
   * @param acts Pointer to an action list
   */
      rule(aggregation_tree * conds,proposed_action_list * acts);
   
   // destructor
   /**
   * Destructor. It deletes precondition and actions too.
   */
      virtual ~rule();
   
   // access to inner variables
   /**
   * Returns a pointer to precondition tree
   * @return Pinter to precondition.
   */
      aggregation_tree * get_conditions();
   
   /**
   * Set a new precondition tree.
   * @param value Pointer to new precondition tree.
   */
      void set_conditions(aggregation_tree * value);
      
   /**
   * Return the number of line at which the rule ends
   */
      unsigned int GetLineNum();

   /**
   * Set the number of line at which the rule ends
   */
      void SetLineNum(unsigned int line);
  
   /**
   * Returns a pointer to action list.
   * @return Pointer to action list
   */
      proposed_action_list * get_actionslist();
      void set_actionslist(proposed_action_list * value);
   
   // get_actions(), given the values of predicates, returns the list of actions with
   // membership_value set to the fuzzy value of the conditions.
   // if either the value of the condition or its reliability is 0, it returns NULL
   /**
   * Given values of predicates, precondition value is evaluated and it is set
   * in all actions and they are returned.
   * @return List of actions with value equal to precondition's one.
   * @param predicates List of predicates for evaluation
   */
      virtual proposed_action_list * get_actions(predicate_list * predicates);
   
   private:
   
   // preconditions of the rule
      aggregation_tree * conditions;
   // postconditions
      proposed_action_list * actionslist;
   // to evaluate predicate
      cando_tree_map * ctm;

      unsigned int mLine;
    };

// containers of rules
/**
 * Instantiation of vetor template to create a rules list.
 * @see rule
 * @see rules_behavior
 * @see rules_file_parser
 */
   typedef vector<rule *> rules_list;

// Class rule_file_parser reads rule from a file
/**
 * This cleass reads from file a list of rules for a rules behavior.
 * Every time a rule is read, it creates a rule object and adds it in a rule base.
 * @short Rules file reader
 * @see rule
 * @see rules_behavior
 * @see rules_list
 */
   class rules_file_parser
   {
   public:
   
   /**
   * Constructor.
   * @return An empty parser.
   */
      rules_file_parser();
   
   // contructor: receives the name of the file to read rules from
   /**
   * Constructor. The object is initialized with the given file name
   * @param afilename File to read rules from.
   * @return A parser obejct for the given file.
   */
      rules_file_parser(char * afilename);
   
   //destructor
   /**
   * Destructor.
   */
      virtual ~rules_file_parser();
   
   // read from the file and return the rules read
   /**
   * It reads, from the file it was initialized with, the list of rules.
   * @return List of rules
   */
      virtual rules_list * read_rules();
   
   private:
      char * filename;
   };

// Class rules_behavior implements a rules behavior
/**
* A behavior based on a rule base. When activated, it evaluates preconditions
* for each rule and proposes related actions.
* @short Rules behavior
* @see rule
* @see rules_file_parser
* @see rules_list
*/
   class rules_behavior: public behavior
   {
   public:
   
   /**
   * Constructor. It creates a new rule behavior with given name. The rule are loaded with given parser.
   * @param name Name of the behavior
   * @param arules_file_parser Rules file parser to use to load rules.
   * @return A behavior with given name and parser.
   */
      rules_behavior(char * name,rules_file_parser * arules_file_parser);
   
   /**
   * Destructor.
   */
      virtual ~rules_behavior();
   
   //redefine do_actions
   /**
   * It evaluates preconditions for each rule and returns proposed actions.
   * @return Proposed action list
   * @param predicates Predicate list to use during evaluation
   */
   proposed_action_list * do_actions(predicate_list * predicates, rules_line_type& rule_lines);
   
   //redefine do_filter_actions
   /**
   *Since it is a behavior of level 1, this method returns null.
   */
   proposed_action_list * filter_actions (predicate_list * predicates, proposed_action_list * aproplist, weight_want_list * want, composer * comp, fuzzyfier * fufy, preacher * priest, defuzzyfier * defufy, rules_line_type& rule_lines);
   
   
   
   private:
   // where the rules are stored
      rules_list * rules;
   // the rules file parser
      rules_file_parser * rules_parser;
   };


   class behavior_filter: public behavior
   
   {
   public:
   
   /**
   * Constructor. It creates a new rule behavior with given name. The rule are loaded with given parser.
   * @param name Name of the behavior
   * @param arules_file_parser Rules file parser to use to load rules.
     @param level, is the level of behavior.
   * @return A behavior with given name and parser.
   */
      behavior_filter(char * name,rules_file_parser * arules_file_parser, int level);
   
   /**
   * Destructor.
   */
      virtual ~behavior_filter();
   
   //redefine do_actions
   /**
   * Simple return null
   */
      virtual proposed_action_list * do_actions(predicate_list * predicates, rules_line_type& rule_lines);
   
   //redefine do_filter_actions
   /**
   * It evaluates preconditions for each rule and returns proposed actions.
   * @return Proposed action list
   * @param predicates Predicate list to use during evaluation
   */
      virtual proposed_action_list * filter_actions(predicate_list * predicates, proposed_action_list * aproplist, weight_want_list * want, composer * comp, fuzzyfier * fufy, preacher * priest, defuzzyfier * defufy, rules_line_type& rule_lines);
   
   /**
   * Create a string, linking the param input.
   */
      char * link_together (char * abehavior, char * aname, char * alabel);
   
   	/**
   	* Translate each proposed action of the input list into a pred beh parent.
   	* Return a list of pred beh parent.
   	*/
      virtual predicate_list * translate (proposed_action_list * aproplist);

   private:
   // where the rules are stored
      rules_list * rules;
   // the rules file parser
      rules_file_parser * rules_parser;
   };


// implementation of get_XXX and set_XXX methods.


   inline aggregation_tree * rule::get_conditions()
   {
      return conditions;
   }

   inline void rule::set_conditions(aggregation_tree * value)
   {
      delete conditions;
      conditions=value;
   }

   inline proposed_action_list * rule::get_actionslist()
   {
      return actionslist;
   }

   inline void rule::set_actionslist(proposed_action_list * value)
   {
      delete actionslist;
      actionslist=value;
   }

   inline void rule::SetLineNum(unsigned int line)
   {
     mLine = line;
   }

   inline unsigned int rule::GetLineNum()
   {
     return mLine;
   }
