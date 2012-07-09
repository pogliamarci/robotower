%{
#include <malloc.h>
#include <string.h>
#include <stdio.h>
#define RULESERROR_VERBOSE 1

extern int ruleslex();
extern int ruleserror(const char *);
extern int rulesleng;

extern void pred_exists(char *name);
extern void data_exists(char * name1, char * name2);
extern void beh_exists(char * name);

extern char* filename;
extern int linenum;

%}

%union {
  char * dataname;
}

%type <dataname> name
%token <dataname> NAME
%token LEFTPAR
%token RIGHTPAR
%token NEWLINE
%token THEN
%token NOT
%token OR
%token AND
%token START
%token SEPARATOR

%token MODALITY
%token SOMEBEHAVIOR


%start ruleslist

%%

ruleslist: /* empty */
           | NEWLINE ruleslist
           | rule NEWLINE  ruleslist
	   | rule
           ;

rule: precond THEN metaactions
    | metaprecond THEN metaactions
      ;

precond: predexpr
         ;
metaprecond: metapredexpr
        ;

predexpr: LEFTPAR NOT predexpr RIGHTPAR
          | LEFTPAR AND predexpr predexpr RIGHTPAR
          | LEFTPAR OR predexpr predexpr RIGHTPAR
          | LEFTPAR name RIGHTPAR       {pred_exists($2);}
	  ;

metapredexpr: LEFTPAR NOT metapredexpr RIGHTPAR
                | LEFTPAR AND metapredexpr metapredexpr RIGHTPAR
                | LEFTPAR AND predexpr metapredexpr RIGHTPAR
                | LEFTPAR AND metapredexpr predexpr RIGHTPAR
                | LEFTPAR OR metapredexpr metapredexpr RIGHTPAR
                | LEFTPAR OR predexpr metapredexpr RIGHTPAR
                | LEFTPAR OR metapredexpr predexpr RIGHTPAR
                | LEFTPAR START name SEPARATOR name SEPARATOR name RIGHTPAR         {beh_exists($3); data_exists($5,$7);}
                | LEFTPAR START SOMEBEHAVIOR SEPARATOR name SEPARATOR name RIGHTPAR {data_exists($5, $7);}
                ;



metaactions: LEFTPAR START MODALITY SEPARATOR name name RIGHTPAR endlist    {data_exists($5,$6);}
             | LEFTPAR START MODALITY SEPARATOR name name RIGHTPAR metaactions    {data_exists($5,$6);}
             | LEFTPAR name name RIGHTPAR endlist    {data_exists($2,$3);}
             | LEFTPAR name name RIGHTPAR metaactions   {data_exists($2,$3);}
                ;



endlist:  /*EMPTY*/ ;

name: NAME {$$=(char *) malloc(strlen($1)+1);
             strcpy($$,$1);}
       ;

%%

int ruleserror(const char *s)
{
  printf("\n# errore nel file %s alla linea %d: %s\n",filename,linenum,s);
  return 0;
}
