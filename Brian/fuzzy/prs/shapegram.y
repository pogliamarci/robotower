%{
#include <getFuzzy.h>//fuzzy/include


#ifdef DMALLOC
#include <dmalloc.h>
#endif

#define YYERROR_VERBOSE 1

extern int shapelex();
int shapeerror(const char *);
extern int shapeleng;
//Added by mr:021128
extern void shape_cancel_memory_leaks();

shapes_list *sl=NULL;
shape * s;
fuzzy_set * fs;
%}

%union {
  char * dataname;
  float number;
}

%type <dataname> name 
%type <number> value
%token <dataname> NAME
%token <dataname> VALUE
%token LEFTPAR
%token RIGHTPAR
%token TRI
%token TOL
%token TOR
%token DIV
%token TRA
%token REC
%token SNG

%start shapelist

%%

shapelist: /*empty*/
           | shape shapelist
           ;

shape: LEFTPAR name setlist RIGHTPAR { s->set_label($2);
                                       sl->add_shape(s);
				       free ($2);
				       s=new shape();}
       ;

setlist: fuzzyset setlist
       | fuzzyset
       ;

fuzzyset: LEFTPAR TRI LEFTPAR name value value value RIGHTPAR RIGHTPAR { s->add_set(new triangle($4,$5,$6,$7));
							                 free ($4);}
        | LEFTPAR TOL LEFTPAR name value value RIGHTPAR RIGHTPAR { s->add_set(new triangle_ol($4,$5,$6));
							           free ($4);}
        | LEFTPAR TOR LEFTPAR name value value RIGHTPAR RIGHTPAR { s->add_set(new triangle_or($4,$5,$6)); 
							           free ($4);}
        | LEFTPAR DIV LEFTPAR name value value value value RIGHTPAR RIGHTPAR { s->add_set(new div_triangle($4,$5,$6,$7,$8));
							                       free ($4);}
        | LEFTPAR TRA LEFTPAR name value value value value RIGHTPAR RIGHTPAR { s->add_set(new trapezium($4,$5,$6,$7,$8));
							                       free ($4);}
        | LEFTPAR REC LEFTPAR name value value RIGHTPAR RIGHTPAR { s->add_set(new rectangle($4,$5,$6));
							           free ($4);}
        | LEFTPAR SNG LEFTPAR name value RIGHTPAR RIGHTPAR { s->add_set(new singleton($4,$5));
							     free ($4);}
        ;

name: NAME {$$=(char *) malloc(strlen($1)+1);
             strcpy($$,$1);}
       ;

value: VALUE {$$=atof($1);}
       ;

%%

int shapeerror(const char *s)
{
  printf("%s\n",s);
  return 0;
}

shapes_list * shape_parser()
{
    sl = new shapes_list();
    s = new shape();
    shapeparse();
    delete s;
    //Added by mr:021128
//    shape_cancel_memory_leaks();
    return sl;
}
