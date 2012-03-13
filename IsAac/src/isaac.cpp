#include <iostream>
#include <string>
#include "brian.h"
#include "Echoes/Sonar.h"
#include "SpyKee/Motion.h"
#include "Vision/Results.h"
#include "ros/ros.h"

#define FUZZYASSOC (char *) "../config/ctof.txt"
#define FUZZYSHAPES (char *) "../config/shape_ctof.txt"
#define PRIES (char *) "../config/Predicate.ini"
#define PRIESACTIONS (char *) "../config/PredicateActions.ini"
#define CANDOES (char *) "../config/Cando.ini"
#define BEHAVIORS (char *) "../config/behaviour.txt"
#define WANTERS (char *) "../config/want.txt"
#define DEFUZZYASSOC (char *) "../config/s_ftoc.txt"
#define DEFUZZYSHAPES (char *) "../config/s_shape.txt"

typedef enum {
	NORTH, SOUTH, EAST, WEST
} CardinalPoint;

class SensorStatus {
	private:
		int sonar_north;
		int sonar_south;
		int sonar_east;
		int sonar_west;
		bool tower_found;
		int tower_position;
	public:
		SensorStatus();
		void fromSonarCallback(const Echoes::Sonar& message);
		void fromVisionCallback(const Vision::Results& message);
		int getSonar(CardinalPoint p);
		/* some getters (declared here as inline) */
		inline bool isTowerDetected() {
			return tower_found;
		}
		inline int getTowerPosition() {
			return tower_position;
		}
};

SensorStatus::SensorStatus()
{
	sonar_north = 0;
	sonar_south = 0;
	sonar_east = 0;
	sonar_west = 0;
	tower_found = false;
	tower_position = 0;
}

void SensorStatus::fromSonarCallback(const Echoes::Sonar& message)
{
	sonar_north = message.north;
	sonar_south = message.south;
	sonar_east = message.east;
	sonar_west = message.west;
}

void SensorStatus::fromVisionCallback(const Vision::Results& message)
{
	tower_found = message.towerFound;
	tower_position = message.towerPos;
}

int SensorStatus::getSonar(CardinalPoint p)
{
	switch(p)
	{
		case NORTH:
			return sonar_north;
			break;
		case SOUTH:
			return sonar_south;
			break;
		case EAST:
			return sonar_east;
			break;
		case WEST:
			return sonar_west;
			break;
		default:
			return 0;
	}
}

class Sender {
	private:
		ros::Publisher motion;
	public:
		Sender(ros::NodeHandle& n);
		void sendMotionMessage(int rot, int tan);
};

Sender::Sender(ros::NodeHandle& n) {
	 motion = n.advertise<SpyKee::Motion>("spykee_motion", 1000);
}

void Sender::sendMotionMessage(int rot, int tan) {
	SpyKee::Motion msg;
	msg.rotSpeed = rot;
	msg.tanSpeed = tan;
	this->motion.publish(msg);
}

void sendBrianOutputs(command_list* cl, Sender& ms) {
	if (cl == NULL)
		return;

	int tan_speed = 0;
	int rot_speed = 0;

	//iteratore
	command_list::iterator it;

	for (it = cl->begin(); it != cl->end(); it++)
	{
		std::string temp = it->first;

		if (temp.compare("TanSpeed") == 0)
		{
			tan_speed = it->second->get_set_point();
		}
		else if (temp.compare("RotSpeed") == 0)
		{
			rot_speed = it->second->get_set_point();
		}
	}

	/* clear list */
	cl->clear();

	cout << "rot_speed: " << rot_speed << " tan_speed: " << tan_speed << endl;

	/* send commands to actuators */
	ms.sendMotionMessage(rot_speed, tan_speed);
}

int main(int argc, char** argv)
{
	SensorStatus sensors;
	crisp_data_list* cdl;

	/* reliability (not used ==> set to 1) */
	const int reliability = 1;

	ros::init(argc, argv, "isaac");
	ros::NodeHandle ros_node = ros::NodeHandle();
	ros::Subscriber sonar_sub = ros_node.subscribe("sonar_data", 1,
			&SensorStatus::fromSonarCallback, &sensors);

	Sender message_sender(ros_node);

	MrBrian brian = MrBrian(FUZZYASSOC, FUZZYSHAPES,
							PRIES, PRIESACTIONS,
							CANDOES, BEHAVIORS,
							WANTERS, DEFUZZYASSOC,
							DEFUZZYSHAPES);

	while (ros::ok())
	{
		/* let's get (a pointer to) the input data list and clear it */
		cdl = (brian.getFuzzy())->get_crisp_data_list();
		cdl->clear();

		/* get some random vars */
		int random_search = rand() % 2;

		cout << "sonar: N " << sensors.getSonar(NORTH) << " , " <<
				sensors.getSonar(SOUTH) << " , " << sensors.getSonar(EAST)
				<< " , " << sensors.getSonar(WEST) << endl;

		/* update inputs */
		cdl->add(new crisp_data("DistanceNorth", sensors.getSonar(NORTH), reliability));
		cdl->add(new crisp_data("DistanceSouth", sensors.getSonar(SOUTH), reliability));
		cdl->add(new crisp_data("DistanceEast", sensors.getSonar(EAST), reliability));
		cdl->add(new crisp_data("DistanceWest", sensors.getSonar(WEST), reliability));

		if (sensors.isTowerDetected())
			cout << "tower detected: pos = " << sensors.getTowerPosition() << endl;

		cdl->add(new crisp_data("TowerDetected", sensors.isTowerDetected(), reliability));
		cdl->add(new crisp_data("TowerPosition", sensors.getTowerPosition(), reliability));

		cout << "random src: " << random_search << endl;
		cdl->add(new crisp_data("RandomSearch", random_search, reliability));

		/* let's start it all */
		brian.run();

		/* parse outputs from brian and send them to actuators */
		sendBrianOutputs(brian.getFuzzy()->get_command_singleton_list(), message_sender);

		ros::spinOnce();
	}
	return 0;
}
