#include <iostream>
#include "brian.h"
#include "Echoes/Sonar.h"
#include "SpyKee/Motion.h"
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
	public:
		int sonar_north;
		int sonar_south;
		int sonar_east;
		int sonar_west;

		SensorStatus();
		void fromSonarCallback(const Echoes::Sonar& message);
		int getSonar(CardinalPoint p);
};

SensorStatus::SensorStatus()
{
	sonar_north = 0;
	sonar_south = 0;
	sonar_east = 0;
	sonar_west = 0;
}

void SensorStatus::fromSonarCallback(const Echoes::Sonar& message)
{
	sonar_north = message.north;
	sonar_south = message.south;
	sonar_east = message.east;
	sonar_west = message.west;
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

int main(int argc, char** argv)
{

	SensorStatus sensors;
	crisp_data_list* cdl;
	command_list* cl;

	/* reliability (not used ==> set to 1) */
	const int reliability = 1;

	ros::init(argc, argv, "isaac");
	ros::NodeHandle ros_node = ros::NodeHandle();
	ros::Subscriber sonar_sub = ros_node.subscribe("sonar_data", 1, &SensorStatus::fromSonarCallback, &sensors);

	Sender message_sender(ros_node);

	MrBrian brian = MrBrian(FUZZYASSOC, FUZZYSHAPES,
							PRIES, PRIESACTIONS,
							CANDOES, BEHAVIORS,
							WANTERS, DEFUZZYASSOC,
							DEFUZZYSHAPES);

	while (ros::ok()) {

		//Ottengo la lista dei dati in ingresso
		cdl = (brian.getFuzzy())->get_crisp_data_list();
		//Pulisco la lista dei dati in ingresso
		cdl->clear();

		// Set crisp vars
		// Distanze dei sonar
		cdl->add(new crisp_data("ObjectDistanceNORD", sensors.getSonar(NORTH), reliability));
		cdl->add(new crisp_data("ObjectDistanceSUD", sensors.getSonar(SOUTH), reliability));
		cdl->add(new crisp_data("ObjectDistanceEST", sensors.getSonar(EAST), reliability));
		cdl->add(new crisp_data("ObjectDistanceOVEST", sensors.getSonar(WEST), reliability));

		brian.run();

		//Leggo le azioni da eseguire
		cl = (brian.getFuzzy())->get_command_singleton_list();

		if (cl != 0)
		{
			int tan_speed = 0;
			int rot_speed = 0;

			//iteratore
			command_list::iterator it;

			for (it = cl->begin(); it != cl->end(); it++)
			{
				string temp = it->first;

				if (temp.compare("TanSpeed") == 0)
				{
					tan_speed = it->second->get_set_point();
				}

				if (temp.compare("RotSpeed") == 0)
				{
					rot_speed = it->second->get_set_point();
				}
			}

			//Cancello la lista delle azioni
			cl->clear();

			message_sender.sendMotionMessage(rot_speed, tan_speed);
		}

		ros::spinOnce();
	}

	return 0;
}
