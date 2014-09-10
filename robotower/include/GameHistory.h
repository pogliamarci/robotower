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

#ifndef GAMEHISTORY_H_
#define GAMEHISTORY_H_

class GameHistory
{
private:
	int won;
	int lost;
	int score;
public:
	GameHistory() : won(0), lost(0), score(0) {};
	inline int getWon()
	{
		return won;
	}
	inline int getLost()
	{
		return lost;
	}
	inline int getScore()
	{
		return score;
	}
	inline void addGame(bool hasWon, int score)
	{
		if(hasWon) won++;
		else lost++;
		this->score += score;
	}
	inline void reset()
	{
		won = lost = score = 0;
	}

};

#endif /* GAMEHISTORY_H_ */
