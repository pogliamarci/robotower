%{
#include <malloc.h>
#include <string.h>
#include <stdio.h>
#define YYERROR_VERBOSE 1

extern int predlex();
int prederror(char *);
extern int shapeleng;

extern char* filename;
extern int linenum;

extern void addpred(char *name);
extern void pred_exists(char *name);
extern void data_exists(char * name1, char * name2);

%}

%union {
  char * dataname;
  int pippo;
}

%type <dataname> name
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

pred: name DEF operator              {addpred($1);}
    ;

operator: LEFTPAR AND operator operator RIGHTPAR
        | LEFTPAR OR operator operator RIGHTPAR
        | LEFTPAR NOT operator RIGHTPAR
        | LEFTPAR D name name RIGHTPAR              {data_exists($3,$4);}
        | LEFTPAR P name RIGHTPAR                         {pred_exists($3);}
        ;

name: NAME  {$$=(char *) malloc(strlen($1)+1);
             strcpy($$,$1);}
      ;

%%

int prederror(char *s)
{
  printf("\n# errore nel file %s alla linea %d: %s\n",filename,linenum,s);
  return 0;
}
