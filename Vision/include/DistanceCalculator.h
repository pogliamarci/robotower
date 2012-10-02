/*
 * RoboTower, Hi-CoRG based on ROS
 *
 * Copyright (C) 2012 Politecnico di Milano
 * Copyright (C) 2012 Marcello Pogliani, Davide Tateo
 * Versione 1.0
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef DISTANCECALCULATOR_CPP_
#define DISTANCECALCULATOR_CPP_

typedef struct s_expWeight
{
	float a;
	float b;
	float c;
	float d;
} ExpWeights;

/* 		=============================
 * 		Curve fitting table used
 * 		=============================
 *
 * 		*=====================================================================================*
 * 		|  distance (m)  | 	factory h 	  |   factory w	   | 	tower h	     |	tower w		  |
 * 		*================*================*================*=================*================*
 * 		|		3		 |		57		  |		 14		   |		  73	 |		13		  |
 * 		|	   2.5		 |		63		  |		 15		   |		  89	 |		17		  |
 * 		|		2		 |		81		  |		 16		   |		 113	 |		20		  |
 * 		|	   1.5		 |	   107		  |		 22		   |		 140	 |		26		  |
 * 		|	   1.3		 |	   123		  |		 25		   |		 159	 |		30		  |
 * 		|		1		 |	   154		  |		 30		   |		 203	 |		36		  |
 * 		|	   0.8		 |	   170		  |		 38		   |		 240	 |		44		  |
 * 		*================*================*================*=================*================*
 *
 */

/*
 * Curve fitting results
 * Using matlab fitting tool, considering exp fitting
 */
const ExpWeights factoryHeightW =
{ 1.086e+05, -0.2161, 4.427, -0.009928 };

const ExpWeights factoryWidthW =
{ 1.971e+04, -0.7035, 3.418,  -0.03903 };

const ExpWeights towerHeightW =
{ 6.639, -0.01443 ,  0.7212, -0.0007514 };

const ExpWeights towerWidthW =
{ 5.5, -0.04783, 0, 0 };

class DistanceCalculator
{
private:
	int distanzaTorre;
	int distanzaFabbrica;
	int larghezzaFabbrica;
	int altezzaFabbrica;
	int altezzaTorre;
	int larghezzaTorre;

public:
	inline int getDistanzaTorre()
	{
		return distanzaTorre;
	}
	inline int getDistanzaFabbrica()
	{
		return distanzaFabbrica;
	}
	void insertDatiTorre(int altezza, int larghezza);
	void insertDatiFabbrica(int altezza, int larghezza);

private:
	int expCalculator(ExpWeights weight, int dim);
};

#endif /* DISTANCECALCULATOR_CPP_ */
