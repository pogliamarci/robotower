%{
#include <malloc.h>
#include <string.h>
#include <stdio.h>
#define YYERROR_VERBOSE 1
  
 extern int asslex();
 extern int asserror(const char *);
 extern int shapeleng;
 
 extern char* filename;
 extern int linenum;
 
 extern void addass(char * name1, char * name2);

%}

%union {
  char * dataname;
  int pippo;
}

%type <dataname> name
%token <dataname> NAME
%token LEFTPAR
%token RIGHTPAR
%token LEVEL
%token VALUE

%start start

%%

start: /*empty*/
     | ass start
     ;

ass: LEFTPAR LEVEL VALUE name name RIGHTPAR         {addass($4,$5);}
     | LEFTPAR name name RIGHTPAR                   {addass($2,$3);}
   ;

name: NAME  {$$=(char *) malloc(strlen($1)+1);
             strcpy($$,$1);}
      ;



%%

int asserror(const char *s)
{
  printf("\n# errore nel file %s alla linea %d: %s\n",filename,linenum,s);
  return 0;
}
