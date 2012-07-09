/***************************************************************************
                          barycentre_defuzzyfier.cpp  -  description
                             -------------------
    begin                : Wed Sep 13 2000
    copyright            : (C) 2000 by Halva Giovanni & Giacomo
    email                : invehalv@airlab.elet.polimi.it
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
//      Module: barycentre_defuzzyfier
//	This module implements the classes used for
//	fuzzyfication and defuzzyfication of data.


#include "barycentre_defuzzyfier.h"
#include "shapes_list.h"
#include "trapezium.h"
#include "point.h"
#include "point_list.h"
#include "shape_point.h"


#include <math.h>


// Class barycentre_defuzzyfier 

barycentre_defuzzyfier::barycentre_defuzzyfier()
  :defuzzyfier()
{
}

barycentre_defuzzyfier::barycentre_defuzzyfier(const barycentre_defuzzyfier &right)
  :defuzzyfier(right)
{
}

barycentre_defuzzyfier::barycentre_defuzzyfier (shape_file_parser *shapefileparser, assoc_file_parser *assocfileparser)
  :defuzzyfier(shapefileparser,assocfileparser)
{
}


barycentre_defuzzyfier::~barycentre_defuzzyfier()
{
}

//## Other Operations (implementation)

command_list * barycentre_defuzzyfier::defuzzyfy (action_list *actionlist)
{
 //algo barycentre_defuzzyfier

  shapes_list * sl = get_shapelist();
  association_list * al = get_assoc();
  command_list * cd =new command_list();
  
  action_list::iterator t;
  t =actionlist->begin();

  while (t != actionlist->end())
    {
      //costruisce la lista ordinata
      shape * curr;
      float zero=0;
      const char * name;
      point_list * pl = new point_list();

      name = (*t).second->get_name();	// get the name of the action
      curr = sl->get_shape((al->get_association(name))->get_shape());  //get the shape related to the action

      pair<action_list::iterator,action_list::iterator> p=actionlist->equal_range(name);
      // get the begin and the end of the action with the same name in the actionlist

	while(p.first != p.second)
	  {
	    float xa,xb,xc,xd,xe,xf,mv;
	    fuzzy_set * cset;

	    cset = curr->get_set((*(p.first)).second->get_label()); //get the fuzzy set;
	    mv = (*(p.first)).second->get_membership_value();
	    xa = (*(trapezium*)cset).get_a();
	    xb = (*(trapezium *)cset).get_b();
	    xc = (*(trapezium *)cset).get_c();
	    xd = (*(trapezium *)cset).get_d();
	    xe = xa+mv*(((*(trapezium *)cset).get_b())-xa);
	    xf = xd+mv*(((*(trapezium *)cset).get_c())-xd);
	    
	    if ( xe > xf ) // se un div_triangle
	      {
		pl->add_point(new shape_point(xa,zero,xe,mv,xb,mv,xb,zero));
		pl->add_point(new shape_point(xc,zero,xc,mv,xf,mv,xd,zero));
	      }
	    else pl->add_point(new shape_point (xa,zero,xe,mv,xf,mv,xd,zero));
	    p.first++; //criterio di avanzamento
	  }

      //fine costruzione lista ordinata
      //inizio costruzione figura
      point_list * npl =new point_list();

      point_list::iterator o; //i
      point_list:: iterator v; //i+l

      float axi,bxi,byi;

      o = pl->begin();//i

      axi = (*(shape_point *)((*o).second)).get_xa();
      bxi = (*(shape_point *)((*o).second)).get_xb();
      byi = (*(shape_point *)((*o).second)).get_yb();

      if (axi < bxi)
	{
	  npl->add_point(new point (axi,zero));
	  npl->add_point(new point (bxi,byi));
	}
      else npl->add_point(new point(bxi,byi));

      while (o!=pl->end()) // costruisce i punti su cui fare la	defuzzyficazione
	{
	  float cxi,dxi;
	 
	  byi=(*(shape_point*)((*o).second)).get_yb();
	  cxi =	(*(shape_point*)((*o).second)).get_xc();
	  dxi=(*(shape_point*)((*o).second)).get_xd();

	  v=++o;// funziona solo cos=EC ma f=E0 anche da criterio di avanzamento
	  
	  if ( v !=pl->end() ) // intersezione con il set successivo
	    {
	      float axi1,bxi1,byi1;

	      axi1 = (*(shape_point *)((*v).second)).get_xa();
	      bxi1 = (*(shape_point *) ((*v).second)).get_xb();
	      byi1 = (*(shape_point *) ((*v).second)).get_yb();

	      if ( dxi > axi1 ) // se le due shape hanno un punto in comune
		{ 
		  float b,c,d,f;

		  b = -axi1*byi1;
		  c = dxi-cxi;
		  d = bxi1-axi1;
		  f = byi*dxi;

		  if ( d*byi-byi1*cxi-b<0)
		    { // intersezione diagonale i+l con piatto i
		      npl->add_point(new point ( (d*byi-b) /byi1,byi));
		      npl->add_point(new point (bxi1, byi1));
		    }
		  else
		    { // intersezione diagonale i con diagonale i+1
		      float x;
		      
		      npl->add_point(new point(cxi,byi));
		      x = (d*f-b*c) / (byi1*c-d*(-byi));
		      npl->add_point (new point (x,((-byi)*x+f)/c));
		      npl->add_point (new point (bxi1,byi1));
		    }
		}
	      else // se le due shape non hanno punti in comune
		{
		  npl->add_point (new point (cxi,byi));
		  if ( dxi != axi1) npl->add_point(new point(dxi,zero));
		  if ( axi1 < bxi1)
		    {
		      npl->add_point (new point (axi1, zero));
		      npl->add_point (new point (bxi1, byi1));
		    }
		  else npl->add_point (new point (bxi1,byi1));
		}
	    }
	  else
	    {
	      if ( cxi < dxi)
		{
		  npl->add_point(new point (cxi,byi));
		  npl->add_point (new point (dxi, zero));
		}
	      else npl->add_point(new point(cxi,byi));
	    }
	}

  //metodo

  float xa,ya, area=0, volume=0, set_point=0;
  float v_t = 0, a_t =0;
  o=npl->begin();
  xa=fabs((*(point *)((*o).second)).get_xa());
  ya=fabs((*(point *)((*o).second)).get_ya());
  v=npl->end();
  v--;
  while(o != v)
    {
      float xb,yb;

      o++;

      xb=fabs((*(point *)((*o).second)).get_xa());
      yb=fabs((*(point *)((*o).second)).get_ya());

      area=(((ya+yb)*fabs(xb-xa))/2);
      if (ya >= yb) volume=yb*(xb*xb-xa*xa)+(ya-yb)*(xa*xa+xb*xb+xa*xb)/3-(ya-yb)*xa*xa;
      else volume=ya*(xb*xb-xa*xa)-(yb-ya)*(xa*xa+xb*xb+xa*xb)/3+(yb-ya)*xb*xb;

      a_t=a_t+area;
      v_t=v_t+volume;

      xa=xb;
      ya=yb;
    }

  set_point=(v_t/(a_t*2));

  cd->add(new command(name,set_point));

  delete pl;
  delete npl;

  t=p.second; //criterio di avanzamento
}

return cd;
}
