#ifndef __STC_H__
#define __STC_H__

#include <stl.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>
/////////////////////////////////////////////////////////
// Definitions of some struct, types, constants
/////////////////////////////////////////////////////////

#define NUMOGG			8

#define tempo			0.1

#define BALL			0
#define IO	       		1
#define ART	       		2
#define OPPONENT		3

#define DIST_WHEELS		37

#ifndef PI
#define PI		       	3.1415
#endif

#define THRESHOLD_DIST	        30
#define THRESHOLD_PRED	        30
#define THRESHOLD_CANDO	        0.6
#define THRESHOLD_PREDICATE	0.6
#define THRESHOLD_END           0.6 
#define NUM_INIT_CYCLES         10
#define NUM_ERRORS		10
#define INIT_OBLIO		0.9	//(1 - (1/NUM_INIT_CYCLES))
#define DISTANCE_TOL	        30
#define ANGULAR_TOL             10
#define ORIENTATION_TOL	        10
#define CLUSTER_DISTANCE        100

#define		_max(x,y)	x>y?x:y
#define		_min(x,y)	x<y?x:y


/////////////////////////////////////////////////////////
struct info
{
  float X;
  float Y;
  float Vx;
  float Vy;
  float alfa;

  float rho;
  float theta;

  float reliability;
  float variance;
};

/*
struct info_rel
{
  float rho;
  float theta;
  float movRel;
  float rotRel;
  float alfaRel;
};
*/

struct tactics
{
  bool SullaPalla;
  bool Zona;
  bool Passaggio;
  bool LanciLunghi;
};


struct ins_seq 
{
  bool operator() (const char *s1, const char *s2) const
  {
 	  if (strcmp(s1,s2) == 0)
		  return false;
	  else
		  return true;
  }
};

#ifndef struct_ins_ord
#define struct_ins_ord

struct ins_ord
{
  bool operator() (const char *s1, const char *s2) const
  {
	/*  if (s1 > s2)
		  return 1;
	  else
		  return 0;*/
	  return strcmp(s1,s2) < 0;
  }

};

#endif

struct less_float
{
  bool operator() (float f1, float f2) const
  {
	  return f1 < f2;
  }

};

template <typename T>
struct bigger
{
    bool operator() (T f1, T f2) const
  {
	  return f1 > f2;
  }

};

struct greater_float
{
  bool operator() (float f1, float f2) const
  {
	  return f1 > f2;
  }

};

struct absol
{
  float X;
  float Y;
  float alfa;
  float reliability;
};

struct rel
{
  float rho;
  float theta;
  float reliability;
};

struct less_absol
{
  bool operator() (absol f1, absol f2) const
  {
      if (f1.X != f2.X)
	  return f1.X > f2.X;
      else
	  return f1.Y > f2.Y;
  }

};
/////////////////////////////////////////////////////////

struct skill
{
  float value;
  float degree;
};

typedef enum AssRel {ASS,REL} ar;

typedef struct pair<char*,char*> strings;

typedef vector<strings> arguments;

typedef map<char*, float, ins_ord> role;

typedef map<const char*, skill,ins_ord> skill_map;

typedef multimap<float,absol,less_float> value_list;

// This struct is used by descructor of class XXXX_list to delete from the memory the objects
// they store
template <class T >
struct destroy_pair_char: public unary_function <T ,void>
{
  void operator() (pair<const char*, T> c)
    {
      if (c.first!=NULL)
	{
	  delete c.first;
	  c.first=NULL;
	}
 
    }
};

template <class T >
struct destroy_pair_char_pointer: public unary_function <T ,void>
{
  void operator() (pair<const char*, T*> c)
    {
      if (c.first!=NULL)
	{
	  delete c.first;
	  c.first=NULL;
	}
      if (c.second!=NULL)
	{
	  delete c.second;
	  c.second=NULL;
	}
  
    }
};


template <class T>
struct destroy_pair_char_const: public unary_function <T,void>
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
struct destroy_pair_float: public unary_function <T,void>
{
  void operator() (pair<const float, T *> c)
    {
//      c.first;
      if (c.second!=NULL)
	{
	  delete c.second;
	  c.second=NULL;
	}
    }
};

#endif
