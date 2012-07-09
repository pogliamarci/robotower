#ifndef data_h
#define data_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include <stl.h>



/**
 * Abstract class implementing the concept of data from
 * outside enviroment.
 * @short abstract data
 */
class data 
{
  public:
    //## Constructors (generated)
      /**
       * Empty constructor
       * @return It returns an empty data
       */
      data();
      
      /**
       * Copy constructor
       * @param right The object to be copied
       * @return It returns a copy of the given data
       */
      data(const data &right);

      /**
       * Constructor
       * @param aname The name to give to data
       * @return It returns an object whose with the given name an reliability set to 1
       */
      data (const char *aname);

      /**
       * Constructor
       * @param aname Name to give to data
       * @param areliability Reliability to give to the data
       * @return It returns an object with given name and reliability
       */
      data (const char *aname, float areliability);

      /**
       * Destructor.
       */
      virtual ~data();

      /**
       * Method to retrieve actual data reliability.
       * @return data reliability; 0 means not reliable.
       */
      float get_reliability ();

      /**
       * Method to set data reliability.
       * @param value New reliability value. 0 means not reliable.
       */
      void set_reliability (float value);

      /**
       * Method to retrieve actual data name.
       * @return A pointer to data name.
       */
      const char * get_name ();
      /**
       * Method to set data name.
       * @param value Pointer to new data name. It must be freed after call.
       */
      void set_name (const char * value);


  private:
    // Data Members for Class Attributes

      float reliability;
      char *name;
};

// Class data

inline float data::get_reliability ()
{
  return reliability;
}

inline void data::set_reliability (float value)
{
  reliability = value;
}

inline const char * data::get_name ()
{
  return name;
}

inline void data::set_name (const char * value)
{
  if (name!=NULL) free(name);
  name=(char *) malloc(strlen(value)+1);
  strcpy(name,value);
}

#endif
