%{
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#define YYERROR_VERBOSE 1

extern int shapelex();
int shapeerror(char *);
extern int shapeleng;

extern char* filename;
extern int linenum;

extern void addshape(char * name);
extern void addfuzzyset(char * name,char * lab);

%}

%union {
  char * dataname;
  //  int pippo;
}

%type <dataname> name
%token <dataname> NAME
%token LEFTPAR
%token RIGHTPAR
%token NEWLINE
%token TOL
%token TRI
%token TRA
%token TOR
%token REC
%token SNG
%token DIV
%token VALUE

%start shapelist

%%

shapelist: /*empty*/
           | shape shapelist
           ;

shape: LEFTPAR name setlist RIGHTPAR {addshape($2);}
     ;

setlist: fuzzyset setlist
       | fuzzyset
       ;

fuzzyset: LEFTPAR TOL LEFTPAR name VALUE VALUE RIGHTPAR RIGHTPAR             {addfuzzyset($4,"TOL");}
        | LEFTPAR TRI LEFTPAR name VALUE VALUE VALUE RIGHTPAR RIGHTPAR       {addfuzzyset($4,"TRI");}
        | LEFTPAR TRA LEFTPAR name VALUE VALUE VALUE VALUE RIGHTPAR RIGHTPAR {addfuzzyset($4,"TRA");}
        | LEFTPAR TOR LEFTPAR name VALUE VALUE RIGHTPAR RIGHTPAR             {addfuzzyset($4,"TOR");}
        | LEFTPAR REC LEFTPAR name VALUE VALUE RIGHTPAR RIGHTPAR             {addfuzzyset($4,"REC");}
        | LEFTPAR SNG LEFTPAR name VALUE RIGHTPAR RIGHTPAR                   {addfuzzyset($4,"SNG");}
        | LEFTPAR DIV LEFTPAR name VALUE VALUE VALUE VALUE RIGHTPAR RIGHTPAR {addfuzzyset($4,"DIV");}
        ;

name: NAME  {$$=(char *) malloc(strlen($1)+1);
             strcpy($$,$1);}
      ;

%%

int shapeerror(char *s)
{
  printf("\n# errore nel file %s alla linea %d: %s\n",filename,linenum,s);
  return 0;
}
