#ifndef crisp_data_list_h
#define crisp_data_list_h 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif

#include <stl.h>

#include "crisp_data_multimap.h"
#include "crisp_data.h"

/**
 * This class collects and manages crisp data. Methods for adding and searching were implemented.
 * @short List of crisp data
 * @see crisp_data
 * @see fuzzyfier
 */
class crisp_data_list : public crisp_data_multimap
{
  public:
      /**
       * Constructor. It simply calls its parent's constructor.
       * @return It returns an empty crisp data list.
       */
      crisp_data_list();
      
      /**
       * Copy constructor. It simply calls its parent's copy constructor.
       * @param right Object to be copied
       * @return It returns a copy of the given crisp data list.
       */
      crisp_data_list(const crisp_data_list &right);

      /**
       * Destructor.
       * It does not delete objects it is pointing to!
       */
      virtual ~crisp_data_list();


      //	Returns the crisp_data object whose name is "name".
      /**
       * It looks in the list for a data with given name.
       * @param name Name of the data to look for.
       * @return It returns a pointer to the desired object, or NULL if not found.
       */
      virtual crisp_data * get_by_name (const char *name);

      /**
       * Add a pointer to a new crisp data in the list.
       * @param acrispdata Pointer to the crisp data to add.
       */
      virtual void add (crisp_data *acrispdata);
	  
      //SM
      virtual void del (crisp_data *acrispdata);

};

#endif
