#include<iostream>
#include"brian.h"

int main()
{
	MrBrian* Brian= new MrBrian(
  					(char *)"../config/ctof.txt",
  	 				(char *)"../config/shape_ctof.txt",
  	 				(char *)"../config/Predicate.ini",
  	 				(char *)"../config/PredicateActions.ini",
  	 				(char *)"../config/Cando.ini",
  	 				(char *)"../config/behaviour.txt",
  	 				(char *)"../config/want.txt",
  	 				(char *)"../config/s_ftoc.txt",
  	 				(char *)"../config/s_shape.txt" 
			           );
	return 0;
}
