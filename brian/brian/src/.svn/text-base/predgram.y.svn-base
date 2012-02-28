%{
#include "engine_objects.h"

#ifdef DMALLOC
#include <dmalloc.h>
#endif

extern int predlex();
int prederror(char *);

aggr_tree_multimap *pt=NULL;

%}

%union {
  char * dataname;
  operation * op;
}

%type <dataname> name
%type <op> operator
%token <dataname> NAME
%token LEFTPAR
%token RIGHTPAR
%token NEWLINE
%token DEF
%token AND
%token OR
%token NOT
%token P
%token D

%start predicates

%%

predicates: pred NEWLINE predicates
          | pred
          | pred NEWLINE
          | NEWLINE predicates
          | NEWLINE
          ;

pred: name DEF operator      {pt->insert(pair < char *, aggregation_tree *> ($1, new aggregation_tree($1,$3)));}
    ;

operator: LEFTPAR AND operator operator RIGHTPAR  {$$=new op_and($3,$4);}
        | LEFTPAR OR operator operator RIGHTPAR   {$$=new op_or($3,$4);}
        | LEFTPAR NOT operator RIGHTPAR           {$$=new op_not($3);}
        | LEFTPAR D name name RIGHTPAR              {$$=new data_node($3,$4); free($3); free($4);}
        | LEFTPAR P name RIGHTPAR                   {$$=new predicate_node($3); free($3);}
        ;

name: NAME  {$$=(char *) malloc(strlen($1)+1);
             strcpy($$,$1);}
      ;

%%
  
int prederror(char *s)
{
  printf("\n# Predicate file error: %s\n",s);
  return 0;
}
 
aggr_tree_multimap * parsefile(aggr_tree_multimap *anaggrmmap)
{
  pt = anaggrmmap;
  predparse();
  return pt;
}
