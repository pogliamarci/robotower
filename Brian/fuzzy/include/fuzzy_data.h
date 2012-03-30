#ifndef fuzzy_data_h
#define fuzzy_data_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#include "data.h"

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include <stl.h>





/**
 * Fuzzy data obtained after fuzzyfication of a crisp data
 * @short Fuzzy Data
 * @see crisp_data
 * @see fuzzy_data_list
 * @see fuzzyfier
 * @see prd_tree_map
 */
class fuzzy_data : public data 
{
  public:
      /**
       * Empty constructor
       * @return It returns an empty fuzzy data
       */
      fuzzy_data();

      /**
       * Copy constructor
       * @param right The object to be copied
       * @return It returns a copy of the given fuzzy data
       */
      fuzzy_data(const fuzzy_data &right);

      /**
       * Constructor
       * @param aname Name of fuzzy data
       * @param alabel Label of fuzzy data
       * @param am_f Membership value of data "aname" w.r.t. fuzzy set "alabel"
       * @return It returns a fuzzy data with given name, label, membership value and reliability set to 1.
       */
      fuzzy_data (const char *aname, const char *alabel, float am_f);

      /**
       * Constructor
       * @param aname Name of fuzzy data
       * @param alabel Label of fuzzy data
       * @param am_f Membership value of data "aname" w.r.t. fuzzy set "alabel"
       * @param areliability Reliability of fuzzy data
       * @return It returns a fuzzy data with given name, label, membership value and reliability.
       */
      fuzzy_data (const char *aname, const char *alabel, float am_f, float areliability);

      /**
       * Destructor
       */
      virtual ~fuzzy_data();

      /**
       * Method to retrieve actual data label
       * @return A pointer to data label.
       */
      const char * get_label ();
      /**
       * Method to set data label.
       * @param value Pointer to new data label. It must be freed after call.
       */
      void set_label (const char * value);

      //	Membership value of the data w.r.t. the set identified
      //	by label.
      /**
       * Method to retrieve membership value.
       * @return data membership value.
       */
      float get_membership_value ();
      /**
       * Method to set membership value.
       * @param value New membership value.
       */
      void set_membership_value (float value);

  private:
      // Class Attributes

      char *label;
      float membership_value;
};


// Class fuzzy_data

inline const char * fuzzy_data::get_label ()
{
  return label;
}

inline void fuzzy_data::set_label (const char * value)
{
  if (label!=NULL) free(label);
  label=(char *) malloc(strlen(value)+1);
  strcpy(label,value);
}

inline float fuzzy_data::get_membership_value ()
{
  return membership_value;
}

inline void fuzzy_data::set_membership_value (float value)
{
  membership_value = value;
}

#endif
