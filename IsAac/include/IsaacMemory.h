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

#ifndef ISAACMEMORY_H_
#define ISAACMEMORY_H_

#include <iostream>

class IsaacMemory
{
public:
	IsaacMemory(int rate) :
			loopRate(rate)
	{
		timeElapsed = 2 * loopRate; //all'inizio sicuramente non ho visto niente
		randomTime = 0; // all'inizio non è passato nulla dall'ultimo set della varaiabile casuale
		towerPosition = 160; //il centro dell'immagine
		randomSearch = 50; //valore neutro per l'insieme fuzzy di ricerca casuale.
	}

	inline void setPosition(int position) //se vedo la torre, salvo la posizione, e dico che l'ho vista
	{
		towerPosition = position;
		timeElapsed = 0;
	}

	int getSearchVariable(int rotSpeed) //calcolo il valore della variabile casuale...
	{
		timeElapsed++;
		if (timeElapsed > 1.5 * loopRate) //se non vedo la torre da troppo tempo... a caso!
		{
			if ((randomTime++) == 4 * loopRate) //per stabilizzare la ricerca casuale (come già facevamo...)
			{
				randomSearch = rand() % 100;
				randomTime = 0;
			}
			return randomSearch;
		}
		else //altrimenti, vado piano dal lato giusto...
		{
			std::cerr << "[SCATTATA]" << std::endl;
			towerPosition -= (timeElapsed == 0) ? 0 : rotSpeed; //se ho appena visto la torre, non aggiorno la sua posizione.
			return (towerPosition < center) ? 45 : 55; //ritorno destra o sinistra a seconda di dove sia la torre.
		}
	}

	void debug()
	{
		std::cerr << "Tower Position: " << towerPosition << std::endl;
		std::cerr << "Credo che la torre sia a: "
				<< ((towerPosition > center) ? "destra" : "sinistra") << std::endl;
		std::cerr << "Non vedo la torre da: " << timeElapsed << " secondi" << std::endl;
		std::cerr << "Allora vado a: " << ((randomSearch > 50) ? "destra" : "sinistra" ) << std::endl;
	}

private:
	const int loopRate;
	static const int center = 160; //rappresenta il centro dell'immagine

private:
	int timeElapsed; //tempo che è passato dall'ultima volta che ho visto al torre
	int towerPosition; //la posizione reale, se settata esplicitamente, supposta, se non ho visto la torre
	int randomTime; //timer per stabilizzare la ricerca casuale
	int randomSearch; //ultimo valore di ricerca casuale

};

#endif /* ISAACMEMORY_H_ */
