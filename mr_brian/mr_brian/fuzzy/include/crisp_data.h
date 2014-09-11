#ifndef crisp_data_h
#define crisp_data_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include <stl.h>

#include "data.h"


//	Crisp data coming from outside enviroment.
/**
 * Crisp data coming from environment.
 * @short Crisp data
 * @see fuzzyfier
 * @see crisp_data_list
 */
class crisp_data : public data
{
  public:
      /**
       * Constructor
       * @return It returns an empty crisp data
       */
      crisp_data();

      /**
       * Copy constructor.
       * @param right Object to be copied.
       * @returns It returns the copy of the given object.
       */
      crisp_data(const crisp_data &right);

      /**
       * Constructor.
       * @param aname Name of the data
       * @param avalue Value of the data
       * @return It returns a crisp data with given name and value, and reliability set to 1
       */
      crisp_data (const char *aname, float avalue);

      //	Constructor: sets name to aname, value to avalue and
      //	reliability to areliability.
      /**
       * Constructor
       * @param aname Name of the data
       * @param avalue Value of the data
       * @param areliability Reliability of the data
       * @return It returns a crisp data with given name, value and reliability
       */
      crisp_data (const char *aname, float avalue, float areliability);

      /**
       * Destructor.
       */
      virtual ~crisp_data();

      /**
       * Method to retrieve data value.
       * @return Data value.
       */
      float get_value ();
      /**
       * Method to set data value.
       * @param value New data value.
       */
      void set_value (float Value);
  private:
    // Data Members for Class Attributes

      float value;
};

inline float crisp_data::get_value ()
{
  return value;
}

inline void crisp_data::set_value (float Value)
{
  value = Value;
}


#endif
