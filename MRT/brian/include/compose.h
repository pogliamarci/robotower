/***************************************************************************
                          compose.h  -  description
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

#ifndef compose_h
#define compose_h 1

#include "interf_obj.h"

#include <getFuzzy.h>

#ifdef DMALLOC
#include <dmalloc.h>
#endif

//  Abstract Class weight_composer: composes a list of actions proposed by different
//  behaviors, in a list of weighted actions.
//  Every child class should implement compose().
//  the returned obejct is equal to "actions" parameter

/**
 * Abstact class used by composer to tune proposed action value with weights coming from wanter
 * @short General want tuning
 * @see composer
 * @see wanter
 */
class weight_composer 
{
  public:
      /**
       * General composition method
       * @return A proposed action list with updated values
       * @param weights Wight from wanter
       * @param actions Actions to update
       */
      virtual proposed_action_list * compose (weight_want_list *weights, proposed_action_list *actions)=0;
};

/**
 * It tunes weights from wanter with proposed actions values multiplying each other.
 * @short Want tuning by multiplication
 * @see composer
 * @see wanter
 */
class mult_weight_composer: public weight_composer
{
 public:
  /**
   * Multiplication composition method. Every action value is compared with the weight of the
   * behavior that proposed it and set to their product.
   * @return The given proposed action list with updated values
   * @param weights Wight from wanter
   * @param actions Actions to update
   */
  proposed_action_list * compose (weight_want_list *weights, proposed_action_list *actions);
};

//  max_weight_composer sets actions' membership value to the maximum between its
//  old value and the relative weight.
/**
 * It takes the maximum between weight and action.
 * @short Want tuning with maximum
 * @see composer
 * @see wanter
 */
class max_weight_composer: public weight_composer
{
 public:
  /**
   * Multiplication composition method. Every action value is compared with the weight of the
   * behavior that proposed it and set to the maximum.
   * @return The given proposed action list with updated values
   * @param weights Wight from wanter
   * @param actions Actions to update
   */
  proposed_action_list * compose (weight_want_list *weights, proposed_action_list *actions);
};

//  min_weight_composer sets actions' membership value to the minimum between its
//  old value and the relative weight.
/**
 * It takes the minimum between weight and action.
 * @short Want tuning with minimum
 * @see composer
 * @see wanter
 */
class min_weight_composer: public weight_composer
{
 public:
  /**
   * Multiplication composition method. Every action value is compared with the weight of the
   * behavior that proposed it and set to the minimum.
   * @return The given proposed action list with updated values
   * @param weights Wight from wanter
   * @param actions Actions to update
   */
  proposed_action_list * compose (weight_want_list *weights, proposed_action_list *actions);
};


// class float_composer is used to compose a list of mebmership value of action in one float value
/**
 * Abstract class used from composer to join together several values coming from actions with same name and label.
 * @short General float composer
 * @see composer
 */
class float_composer
{
 public:
  /**
   * Pure virtual method for float composing.
   * @return Composed value
   * @param floatlist List of values to compose
   */
  virtual float compose(vector<float> *floatlist)=0;
};

// class min_float_composer: selects the minimum between passed values
/**
 * Is selects the minimum between values composer gave it
 * @short Minimum float composer
 * @see composer
 */
class min_float_composer: public float_composer
{
 public:
  /**
   * It gives back the minimum between the given values.
   * @return the minimum in floatlist
   * @param floatlist Values to search in.
   */
  float compose(vector<float> *floatlist);
};


// class max_float_composer: selects the maximum between passed values

/**
 * Is selects the maximum between values composer gave it
 * @short Maximum float composer
 * @see composer
 */
class max_float_composer: public float_composer
{
 public:
  /**
   * It gives back the maximum between the given values.
   * @return the maximum in floatlist
   * @param floatlist Values to search in.
   */
  float compose(vector<float> *floatlist);
};

// class average_float_composed: calculates the average of passed values

/**
 * Is evalues the average between values composer gave it
 * @short Average float composer
 * @see composer
 */
class average_float_composer: public float_composer
{
 public:
  /**
   * It gives back the average between the given values.
   * @return the average in floatlist
   * @param floatlist Values to search in.
   */
  float compose(vector<float> *floatlist);
};


//CLASSE COMPOSER
// Class composer: composes the list of proposed_action in a list of actions.
/**
 * It deals with composing actions between themselves and want conditions. To achieve this task,
 * it uses two classes: weight_want_composer, for composing want with actions, and float_composer
 * for actions between themselves.
 * @short Action composer
 * @see weight_want_composer
 * @see float_composer
 * @see behavior_engine
 * @see wanter
 * @see defuzzyfier
 */
class composer
{
 public:

  composer();
  
  /**
   * Constructor. Parameters are used to specify which sub-composers must be used.
   * @return A composer with desired sub-composers
   * @param aweight_composer A pointer to a weight_composer object to use for composing want and actions
   * @param afloat_composer A pointer to a float_composer object to use for composing actions with themselves.
   */
  composer(weight_composer * aweight_composer, float_composer * afloat_composer);

  /**
   * Destructor
   */
  virtual ~composer();

  // compose() composes proposed actions with same name and label in one action
  // whose membership value is the maximum of the first actions.
  /**
   * It first composes proposed actions list with weights from wanter, and then joins together
   * all actions with same name and labels.
   * @return An action list containing only one (or none) action with given name and label.
   * @param weights Weight from wanter
   * @param actions Proposed actions from behavior engine
   */
  action_list* compose(weight_want_list *weights, proposed_action_list *actions);

 private:
  weight_composer *w_c;
  float_composer *f_c;
};

#endif
