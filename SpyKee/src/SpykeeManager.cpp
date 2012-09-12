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

#include "SpykeeManager.h"
#include <iostream>
#include <exception>

// #define DEBUG_SPYKEE

#define PACKET_TYPE_NONE				0
#define PACKET_TYPE_AUDIO				1
#define PACKET_TYPE_VIDEO				2
#define PACKET_TYPE_POWER				3
#define PACKET_TYPE_LED					4
#define PACKET_TYPE_MOVE				5
#define PACKET_TYPE_FILE				6
#define PACKET_TYPE_PLAY				7
#define PACKET_TYPE_STOP				8
#define PACKET_TYPE_VOLUME				9
#define PACKET_TYPE_AUTH_REQUEST		10
#define PACKET_TYPE_AUTH_REPLY			11
#define PACKET_TYPE_MUTE				12
#define PACKET_TYPE_CONFIG				13
#define PACKET_TYPE_WIRELESS_NETWORKS	14
#define PACKET_TYPE_STREAMCTL			15
#define PACKET_TYPE_ENGINE				16	// engine packets subtype can be found in Engine.h
#define PACKET_TYPE_LOG					17
#define PACKET_TYPE_MOTOR_SPEED			18

// use with PACKET_TYPE_STREAMCTL, from point of view of robot
#define STREAM_ID_NONE		0
#define STREAM_ID_VIDEO		1
#define STREAM_ID_AUDIO_IN	2	// => mic
#define STREAM_ID_AUDIO_OUT	3	// => speaker

// (part of) sub-types of PACKET_TYPE_ENGINE
#define MESSAGE_TYPE_CHARGE_STOP		5

using namespace std;

SpykeeManager::SpykeeManager(string username, string password) throw(SpykeeException) :
		bufferContentLength(0), hasNewImage(false), lastBatteryLevel(0), batteryIsUnread(false)
{

	const char discoveryMsg[] = { 'D', 'S', 'C', 'V', 1 };
	char respString[100];
	string sourceAddress;
	unsigned short port;

	try
	{
		/* search for a robot and get its address */
		UDPSocket udp;
		udp.sendTo(discoveryMsg, 5, "172.17.6.255", 9000);

		if (udp.recvFrom(respString, 100, sourceAddress, port) > 0)
			cout << (respString + 6) << endl;

		/* start TCP connection with the robot */
		tcp = new TCPSocket(sourceAddress, 9000);

		/* authentication */
		authenticate(username, password);
		if (tcp->recv(respString, 100) > 0)
			cout << (respString + 7) << endl;

	}
	catch (SocketException& e)
	{
		throw SpykeeException();
	}

	#ifdef DEBUG_SPYKEE
	cerr << "Connessione effettuata" << endl;
	#endif
}

void SpykeeManager::authenticate(string username, string password)
{
	const char payloadLength = username.length() + password.length() + 2;
	int8_t messageAuthentication[payloadLength];
	messageAuthentication[0] = username.length();
	for (unsigned int i = 0; i < username.length(); i++)
	{
		messageAuthentication[1 + i] = username.at(i);
	}

	messageAuthentication[1 + username.length()] = password.length();
	for (unsigned int i = 0; i < password.length(); i++)
	{
		messageAuthentication[2 + username.length() + i] = password.at(i);
	}
	sendMsg(PACKET_TYPE_AUTH_REQUEST, payloadLength, messageAuthentication);
}

void SpykeeManager::setCameraStatus(bool setEnabled)
{
	const int8_t payload[] = {STREAM_ID_VIDEO, (int8_t) (setEnabled ? 1 : 0)};
	sendMsg(PACKET_TYPE_STREAMCTL, 2, payload);

	#ifdef DEBUG_SPYKEE
	cout << "Camera accesa" << endl;
	#endif
}

void SpykeeManager::readPacket()
{
	readDataFromSpykee();
	char type = PACKET_TYPE_NONE;
	if(bufferContentLength >= 5) type = getCurrentMessageType();
	switch(type)
	{
	case PACKET_TYPE_VIDEO:
		hasNewImage = true;
		break;
	case PACKET_TYPE_POWER:
		batteryIsUnread = true;
		lastBatteryLevel = buffer[5];
		break;
	default:
#ifdef DEBUG_SPYKEE
		cerr << "Messaggio non gestito di tipo " << (int) type << endl;
#endif
		break;
	}
}

void SpykeeManager::readDataFromSpykee()
{
	bufferContentLength = tcp->recv(buffer, SPYKEE_MAX_IMAGE);
	if(bufferContentLength > SPYKEE_MAX_IMAGE)
		bufferContentLength = SPYKEE_MAX_IMAGE;
}

/* returns a pointer to a dynamically allocated vector containing the image data */
/* waits the image to be sent from the robot... if no img is sent, the method doesn't return */
vector<unsigned char>* SpykeeManager::readImage()
{
	bool hasCompletedAcquisition = false;
	unsigned int current_position = 0;

	if(!hasNewImage) // this func should not be called here...
		throw new exception(); // TODO refactor!

	hasNewImage = false;

	unsigned int image_length = getCurrentPayloadSize();
	vector<unsigned char>* image_data = new vector<unsigned char>(image_length);
	current_position = 0;

	#ifdef DEBUG_SPYKEE
	cerr << "New image found. Size: " << image_length << " byte" << endl;
	#endif

	while (!hasCompletedAcquisition)
	{
		#ifdef DEBUG_SPYKEE
		cerr << "Processing a new packet. Size: " << message_size << " byte." << endl;
		#endif

		/* skip first 5 bytes (header), already processed in the NOT_FOUND status */
		register int i = current_position == 0 ? 5 : 0;
		for (;(i < bufferContentLength) && (current_position < image_length); i++)
		{
			image_data->at(current_position) = buffer[i];
			current_position++;
		}
		if (current_position >= image_length)
			break;
		else readDataFromSpykee();

		#ifdef DEBUG_SPYKEE
		cerr <<"Finished processing the packet. Current buffer position = " << current_position << endl;
		cerr <<"Remaining bytes in the image : " << (image_length - current_position) << endl;
		#endif
	}
	return image_data;
}

int SpykeeManager::getCurrentPayloadSize()
{
	return (buffer[3] << 8) + buffer[4];
}

char SpykeeManager::getCurrentMessageType()
{
	if( (buffer[0] == 'P') && (buffer[1] == 'K') )
		return buffer[2];
	else return PACKET_TYPE_NONE;
}

void SpykeeManager::move(char leftSpeed, char rightSpeed)
{
	int8_t payload[] = { leftSpeed, rightSpeed };
	sendMsg(PACKET_TYPE_MOVE, 2, payload);
}

void SpykeeManager::setLed(char ledid, bool status)
{
	int8_t statcmd = status ? 1 : 0;
	const int8_t payload[] = {ledid, statcmd};
	sendMsg(PACKET_TYPE_LED, 2, payload);
}

void SpykeeManager::unplug()
{
	const int8_t payload[] = {MESSAGE_TYPE_CHARGE_STOP};
	sendMsg(PACKET_TYPE_ENGINE, 1, payload);
}

void SpykeeManager::sendMsg(uint8_t pkType, int length, const int8_t* payload)
{
	char header[5];
	header[0] = 'P';
	header[1] = 'K';
	header[2] = pkType;
	header[3] = length >> 8;
	header[4] = length;
	tcp->send(header, 5);
	tcp->send(payload, length);
}
