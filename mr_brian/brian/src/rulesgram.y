%{
#include "interf_obj.h"
#include "rules_behav.h"
#include "preacher.h"

#ifdef DMALLOC
#include <dmalloc.h>
#endif

#define YYERROR_VERBOSE 1

extern int yylex();
int yyerror(const char *);
extern int yyleng;

rules_list *rl=NULL;
proposed_action_list *actuallist=NULL;
aggregation_tree *ptrule=NULL;

 unsigned int line_num;

%}

%union {
  char * dataname;
  rule * r;
  aggregation_tree * prec;
  operation * op;
}

%type <op> predexpr
%type <dataname> name
%token <dataname> NAME
%token LEFTPAR
%token RIGHTPAR
%token NEWLINE
%token THEN
%token NOT
%token OR
%token AND

%start ruleslist

%%

ruleslist: /* empty */
           | rule  ruleslist
           ;

rule: precond THEN actions NEWLINE    { 
					rule* r = new rule(ptrule,actuallist);
					r->SetLineNum(line_num);
					rl->push_back(r);
					actuallist=new proposed_action_list();
					ptrule=new aggregation_tree();}
      ;

precond: predexpr       {ptrule->set_pTerm($1);}
         ;

predexpr: LEFTPAR NOT predexpr RIGHTPAR         {$$=new op_not($3);}
        | LEFTPAR AND predexpr predexpr RIGHTPAR    {$$=new op_and($3,$4);}
        | LEFTPAR OR predexpr predexpr RIGHTPAR {$$=new op_or($3,$4);}
        | LEFTPAR name RIGHTPAR       {$$=new predicate_node($2); free($2);}
	;

actions: LEFTPAR name name RIGHTPAR endlist        {actuallist->add(new proposed_action($2,$3,"noname"));
                                                      free($2);
	                                              free($3);}
        | LEFTPAR name name RIGHTPAR actions   {actuallist->add(new proposed_action($2,$3,"noname"));
                                                   free($2);
                                                   free($3);}
         ;

endlist:  /*EMPTY*/ ;

name: NAME {$$=(char *) malloc(strlen($1)+1);
             strcpy($$,$1);}
       ;

%%

int yyerror(const char *s)
{
  printf("%s\n",s);
  return 0;
}

rules_list * parser()
{
  line_num = 1;
  rl=new rules_list();
  actuallist=new proposed_action_list();
  ptrule=new aggregation_tree();
  yyparse();
  delete actuallist;
  delete ptrule;
  return rl;
}
