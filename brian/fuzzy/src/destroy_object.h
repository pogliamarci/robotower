#ifndef destroy_object_h
#define destroy_object_h 1


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#ifdef DMALLOC

#include <dmalloc.h>

#endif


// stl
#include <stl.h>


// This struct is used by descructor of class XXXX_list to delete from the memory the objects
// they store

/**
 * It is used to destroy object in all container classes.
 * For example, if you have a crisp data list, to destroy it type:
 *
 * for_each(crisp_data_list_pointer->begin(),crisp_data_list_pointer->end(),destroy_object<crisp_data>());
 */
template <class T>
struct destroy_object: public unary_function <T,void>
{
  void operator() (pair<const char*, T *> c)
    {
      c.first=NULL;
      if (c.second!=NULL)
	{
	  delete c.second;
	  c.second=NULL;
	}
    }
};

template <class T>
struct destroy_object_first: public unary_function <T,void>
{
  void operator() (pair<const char*, T *> c)
    {
      free((char*)c.first);
      c.first=NULL;
    }
};

template <class T>
struct destroy_object_complete: public unary_function <T,void>
{
  void operator() (pair<const char*, T *> c)
    {
      free((char*)c.first);
      c.first=NULL;
      if (c.second!=NULL)
	{
	  delete c.second;
	  c.second=NULL;
	}
    }
};

template <class C>
struct destroy_point_list: public unary_function<C,void>
{
  void operator() (pair<float, C *> c)
  {
    if(c.second!=NULL)
      delete c.second;
  }
};

#endif
