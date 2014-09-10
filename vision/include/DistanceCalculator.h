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

const ExpWeights factoryHeightW = {7.527, -0.03944, 2.278, -0.007077};
const ExpWeights factoryWidthW = {5.111, -0.1401, 1.939 , -0.02379 };

const ExpWeights towerHeightW = {6.628, -0.03021, 2.694, -0.006427};
const ExpWeights towerWidthW = {7.008, -0.07823, 0.895, -0.01019};

class DistanceCalculator {
private:
	int distanzaTorrre;
	int distanzaFabbrica;
	int larghezzaFabbrica;
	int altezzaFabbrica;
	int altezzaTorre;
	int larghezzaTorre;

public:
	inline int getDistanzaTorre()
	{
		return distanzaTorrre;
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
