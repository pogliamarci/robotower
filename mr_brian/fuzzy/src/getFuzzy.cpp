#include "getFuzzy.h"
#include <iostream>


GetFuzzy::GetFuzzy()
{
  acl = new action_list();
  cml_sin = new command_list();
  cml_bry = new command_list();
  cdl = new crisp_data_list();
  fdl = new fuzzy_data_list();

  fufy = new fuzzyfier();
  singleton = new singleton_defuzzyfier();
  barycentre = new barycentre_defuzzyfier();
}

GetFuzzy::GetFuzzy(char * fuzzyassoc, 
                   char * fuzzyshapes, 
                   char * defuzzyassoc, 
                   char * defuzzyshapes)
{
  acl = new action_list();
  cout<<"\naction_list";

  cml_sin = new command_list();
  cout<<"\ncommand_list(singleton)";

  cml_bry = new command_list();
  cout<<"\ncommand_list(barycentre)";

  cdl = new crisp_data_list();
  cout<<"\ncrisp_data_list";

  fdl = new fuzzy_data_list();
  cout<<"\nfuzzy_data_list";

  //fuzzyfier
  assoc_file_parser * ctof = new assoc_file_parser(fuzzyassoc);
  shape_file_parser * shaperctof = new shape_file_parser(fuzzyshapes);
  fufy = new fuzzyfier(shaperctof,ctof);
  cout<<"\nfuzzyfier";

  assoc_file_parser * ftoc = new assoc_file_parser(defuzzyassoc);
  shape_file_parser * shaperftoc = new shape_file_parser(defuzzyshapes);
  singleton = new singleton_defuzzyfier(shaperftoc,ftoc);
  cout<<"\nsingleton_pippo";
}

action_list * GetFuzzy::get_action_list()
{
  return acl;
}

command_list * GetFuzzy::get_command_singleton_list()
{
  return cml_sin;
}

command_list * GetFuzzy::get_command_barycentre_list()
{
  return cml_bry;
}

crisp_data_list * GetFuzzy::get_crisp_data_list()
{
  return cdl;
}

fuzzy_data_list * GetFuzzy::get_fuzzy_data_list()
{
  return fdl;
}

fuzzyfier * GetFuzzy::get_fuzzyfier()
{
  return fufy;
}

singleton_defuzzyfier * GetFuzzy::get_singleton()
{
  return singleton;
}

barycentre_defuzzyfier * GetFuzzy::get_barycentre()
{
  return barycentre;
}

void GetFuzzy::flush()
{
  for_each(acl->begin(), acl->end(), destroy_object<action>());
  for_each(cml_sin->begin(), cml_sin->end(), destroy_object<command>());
  for_each(cml_bry->begin(), cml_bry->end(), destroy_object<command>());
  for_each(cdl->begin(), cdl->end(), destroy_object<crisp_data>());
  for_each(fdl->begin(), fdl->end(), destroy_object<fuzzy_data>());

  acl->clear();
  cml_sin->clear();
  cml_bry->clear();
  cdl->clear();
  fdl->clear();

  delete acl;
  delete cml_sin;
  //  delete cml_bry;
  //  delete cdl;
  delete fdl;

  //  acl = new action_list();
  //  cml_sin = new command_list();
  //  cml_bry = new command_list();
  //  cdl = new crisp_data_list();
  //  fdl = new fuzzy_data_list();
}

void GetFuzzy::debug()
{
  viewcdl(cdl);
  viewfdl(fdl);
  viewacl(acl);
  viewcdl(cml_sin);
}

GetFuzzy::~GetFuzzy()
{
  delete acl;
  delete cml_sin;
  delete cml_bry;
  delete cdl;
  delete fdl;
  delete fufy;
  delete singleton;
  delete barycentre;
}

void GetFuzzy::set_action_list(action_list * actionlist)
{
  acl=actionlist;
}

void GetFuzzy:: set_command_singleton_list(command_list * csl)
{
  cml_sin=csl;
}

void GetFuzzy:: set_command_barycentre_list(command_list * cbl)
{
  cml_bry=cbl;
}

void GetFuzzy:: set_crisp_data_list(crisp_data_list * crisplist)
{
  cdl=crisplist;
}

void GetFuzzy::set_fuzzy_data_list(fuzzy_data_list * fuzzylist)
{
  fdl=fuzzylist;
}

