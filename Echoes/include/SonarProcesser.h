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

#ifndef SONARPROCESSER_H_
#define SONARPROCESSER_H_

#include "Processer.h"
#include "ros/ros.h"
#include "Echoes/Sonar.h"
#include <vector>

class SonarProcesser : public Processer
{
	public:
		SonarProcesser(ros::Publisher pub);
		void process(string str);
		~SonarProcesser();
	private:
		ros::Publisher sonar_data_pub;
		int north;
		int south;
		int east;
		int west;
		void publishLast();
		void tokenize(const std::string& str,
		                std::vector<std::string>& tokens,
		                const std::string& delimiters);
};

#endif /* SONARPROCESSER_H_ */
