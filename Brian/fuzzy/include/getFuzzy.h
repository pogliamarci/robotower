#ifndef getFuzzy_h
#define getFuzzy_h 1

#include "action.h"
#include "action_multimap.h"
#include "action_list_debug.h"
#include "action_list.h"

#include "assoc_file_parser.h"
#include "association.h"
#include "association_list.h"
#include "association_set_multimap.h"
#include "barycentre_defuzzyfier.h"

#include "command.h"
#include "command_multimap.h"
#include "command_list_debug.h"
#include "command_list.h"

#include "crisp_data.h"
#include "crisp_data_list_debug.h"
#include "crisp_data_list.h"
#include "crisp_data_multimap.h"
#include "data.h"
#include "defuzzyfier.h"
#include "destroy_object.h"
#include "div_triangle.h"
#include "fuzzy_crisp_rel.h"
#include "fuzzy_data.h"
#include "fuzzy_data_list_debug.h"
#include "fuzzy_data_list.h"
#include "fuzzy_data_multimap.h"
#include "fuzzyfier.h"
#include "fuzzy_set.h"
#include "fuzzy_set_multimap.h"
#include "ltstr.h"
#include "ord.h"
#include "point.h"
#include "point_list.h"
#include "point_multimap.h"
#include "rectangle.h"
#include "shape_file_parser.h"
#include "shape.h"
#include "shape_multimap.h"
#include "shape_point.h"
#include "shapes_list.h"
#include "singleton_defuzzyfier.h"
#include "singleton.h"
#include "trapezium.h"
#include "triangle.h"
#include "triangle_ol.h"
#include "triangle_or.h"


class GetFuzzy
{
 private:

  action_list * acl;
  command_list * cml_sin;
  command_list * cml_bry;
  crisp_data_list * cdl;
  fuzzy_data_list * fdl;

  fuzzyfier * fufy;
  singleton_defuzzyfier * singleton;
  barycentre_defuzzyfier * barycentre;

 public:

  GetFuzzy();
  GetFuzzy(char * fuzzyassoc, 
           char * fuzzyshapes, 
           char * defuzzyassoc, 
           char * defuzzyshapes);
  fuzzyfier * get_fuzzyfier();
  singleton_defuzzyfier * get_singleton();
  barycentre_defuzzyfier * get_barycentre();

  void set_action_list(action_list * actionlist);
  void set_command_singleton_list(command_list * csl);
  void set_command_barycentre_list(command_list * cbl);
  void set_crisp_data_list(crisp_data_list * crisplist);
  void set_fuzzy_data_list(fuzzy_data_list * fuzzylist);

  action_list * get_action_list();
  command_list * get_command_singleton_list();
  command_list * get_command_barycentre_list();
  crisp_data_list * get_crisp_data_list();
  fuzzy_data_list * get_fuzzy_data_list();

  void flush();

  void debug();

  ~GetFuzzy();

};

#endif
