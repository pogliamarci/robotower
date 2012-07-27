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

#include "DistanceCalculator.h"
#include <cmath>

void DistanceCalculator::insertDatiFabbrica(int altezza, int larghezza)
{
	larghezzaFabbrica = larghezza;
	altezzaFabbrica = altezza;
	distanzaFabbrica = expCalculator(weightsAltezzaF,altezzaFabbrica);

}

void DistanceCalculator::insertDatiTorre(int altezza, int larghezza)
{
	larghezzaTorre = larghezza;
	altezzaTorre = altezza;
	distanzaFabbrica = expCalculator(weightsAltezzaT,altezzaFabbrica);
}


int DistanceCalculator::expCalculator(ExpWeights weight, int dim)
{
	return (int) weight.a*exp(weight.b*dim)+weight.c*exp(weight.d*dim);
}



