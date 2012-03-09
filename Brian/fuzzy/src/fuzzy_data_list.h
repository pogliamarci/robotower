#ifndef fuzzy_data_list_h
#define fuzzy_data_list_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include <stl.h>

#include "fuzzy_data.h"
#include "fuzzy_data_multimap.h"

/**
 * This class collects and manages fuzzy data. Methods for adding and searching were implemented.
 * @short List of fuzzy data
 * @see fuzzy_data
 * @see fuzzyfier
 * @see prd_tree_map
 */
class fuzzy_data_list : public fuzzy_data_multimap
{
  
 public:

  /**
   * Constructor. It simply calls its parent's constructor.
   * @return It returns an empty fuzzy data list.
   */
  fuzzy_data_list();
  
  /**
   * Copy constructor. It simply calls its parent's copy constructor.
   * @param right Object to be copied
   * @return It returns a copy of the given fuzzy data list.
   */
  fuzzy_data_list(const fuzzy_data_list &right);

  /**
   * Destructor.
   * It does not delete objects it is pointing to!
   */
  virtual ~fuzzy_data_list();
  
  //        Sice fuzzy_data_list is a multimap, and all the fuzzy_data
  //        returned have the same name, the key in the retruned object is
  //        fuzzy_data::label
  /**
   * This method is used to search some fuzzy data in a list. Since there can
   * be more than one fuzzy data with the same name, all of them are returned
   * in another fuzzy data list. When you finished using the returned object,
   * you must delete it but not the pointed objects!
   * 
   * @param name Name too look for
   * @return a fuzzy data list whose elements have all the given name. The list is indexed by label. If no element is found, an empty list is returned.
   */
  virtual fuzzy_data_list * get_by_name (const char *name);
  
  /**
   * It looks into the list for a data with given name and label
   * @param name Data name to look for.
   * @param label Data label to look for.
   * @return a pointer to the desired fuzzy data, NULL otherwise.
   */
  virtual fuzzy_data * get_by_name_label (const char *name, const char *label);
  
  //	Add a fuzzy_data object into the list. Return value:
  //	not specified.
  /**
   * Add a pointer to a new fuzzy data in the list.
   * @param afuzzydata Pointer to the fuzzy data to add.
   */
  virtual void add (fuzzy_data *afuzzydata);
};

#endif
