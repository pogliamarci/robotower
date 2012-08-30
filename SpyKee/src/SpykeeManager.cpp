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
#include <vector>
#include <iostream>
#include <stdio.h>
#include <string.h>

/* #define DEBUG_SPYKEE */
#define SPYKEE_MAX_IMAGE 10000

using namespace std;

SpykeeManager::SpykeeManager(char* username, char* password) throw(SpykeeException)
{
	#ifdef DEBUG_SPYKEE
	cout << "Nome utente: " << username << endl;
	cout << "Password: " << password << endl;
	#endif

	/* initialize variables to connect */
	const char UDPMessage[] = { 68, 83, 67, 86, 01 };
	const char message[] = { 80, 75, 10, 00, 12 };

	UDPSocket udp;
	char respString[100];
	char messageAuthentication[strlen(username) + strlen(password) + 2];
	string sourceAddress;
	unsigned short port;

	/* build authentication message */
	messageAuthentication[0] = 5;
	for (unsigned int i = 0; i < strlen(username); i++)
	{
		messageAuthentication[1 + i] = username[i];
	}

	messageAuthentication[1 + strlen(username)] = 5;
	for (unsigned int i = 0; i < strlen(password); i++)
	{
		messageAuthentication[2 + strlen(username) + i] = password[i];
	}

	/* connection */
	try {
		/* search for a turned on robot and gets its address */
		udp.sendTo(UDPMessage, 5, "172.17.6.255", 9000);

		if (udp.recvFrom(respString, 100, sourceAddress, port) > 0)
			printf("%s\n", (respString + 7));

		/* start TCP connection with the robot */
		tcp = new TCPSocket(sourceAddress, 9000);
		tcp->send(message, 5);

		/* authentication */
		tcp->send(messageAuthentication, strlen(username) + strlen(password) + 2);
		if (tcp->recv(respString, 100) > 0)
			printf("%s\n", (respString + 7));

	} catch (SocketException& e) {
		#ifdef DEBUG_SPYKEE
		cerr << e.what() << endl;
		#endif
		throw SpykeeException();
	}

	#ifdef DEBUG_SPYKEE
	cout << "Connessione effettuata" << endl;
	#endif
}

void SpykeeManager::startCamera()
{
	const char firstMessage[] = { 80, 75, 15, 0, 2 };
	const char secondMessage[] = { 1, 1, 80, 75, 15, 0, 2, 2, 1 };
	tcp->send(firstMessage, 5);
	tcp->send(secondMessage, 9);

	#ifdef DEBUG_SPYKEE
	cout << "Camera accesa" << endl;
	#endif
}

/* returns a pointer to a dynamically allocated vector containing the image data */
/* waits the image to be sent from the robot... if no img is sent, the method doesn't return */
vector<unsigned char>* SpykeeManager::getImage()
{
	enum ImageStatuses { NOT_FOUND, ACQUIRING, COMPLETELY_ACQUIRED };
	ImageStatuses image_status = NOT_FOUND;
	unsigned char buffer[SPYKEE_MAX_IMAGE];
	vector<unsigned char>* image_data = NULL;

	unsigned int current_position = 0;
	unsigned int image_length = 0;

	while (image_status != COMPLETELY_ACQUIRED)
	{
		int message_size;
		message_size = tcp->recv(buffer, SPYKEE_MAX_IMAGE);
		message_size = (message_size > SPYKEE_MAX_IMAGE) ? SPYKEE_MAX_IMAGE : message_size;
		#ifdef DEBUG_SPYKEE
		cout << "Processing a new packet. Size: " << message_size << " byte." << endl;
		#endif

		/* if we are looking for a new image, check whether it's arrived */
		if (image_status == NOT_FOUND && containsNewImage(buffer))
		{
			/* Processing image metadata. By setting the status to ACQUIRING we will enter
			 * the next if branch, that will process the image bytes of the packet */
			image_status = ACQUIRING;
			image_length = getImageSize(buffer);
			image_data = new vector<unsigned char>(image_length);
			current_position = 0;

			#ifdef DEBUG_SPYKEE
			cout << "New image found. Size: " << image_length << " byte" << endl;
			#endif
		}

		if (image_status == ACQUIRING)
		{
			#ifdef DEBUG_SPYKEE
			cout << "Acquiring image. Current buffer position = " << current_position << endl;
			#endif

			register int i = current_position == 0 ? 5 : 0; /* skip first 5 bytes (metadata), already processed in the NOT_FOUND status */
			for (;(i < message_size) && (current_position < image_length); i++)
			{
				image_data->at(current_position) = buffer[i];
				current_position++;
			}
			if (current_position >= image_length)
			{
				image_status = COMPLETELY_ACQUIRED;
			}

			#ifdef DEBUG_SPYKEE
			cout <<"Finished processing the packet. Current buffer position = " << current_position << endl;
			cout <<"Remaining bytes in the image : " << (image_length - current_position) << endl;
			#endif
		}
	}
	return image_data;
}

int SpykeeManager::getImageSize(unsigned char buffer[SPYKEE_MAX_IMAGE])
{
	return (buffer[3] << 8) + buffer[4];
}

bool SpykeeManager::containsNewImage(unsigned char buffer[SPYKEE_MAX_IMAGE])
{
	return (buffer[0] == 80) && (buffer[1] == 75) && (buffer[2] == 2);
}

void SpykeeManager::move(char leftSpeed, char rightSpeed)
{
	const char message[] = { 80, 75, 5, 0, 2 };
	char speedMessage[] = { leftSpeed, rightSpeed, 0 };

#ifdef DEBUG_SPYKEE
	cout << "LeftMotor: " << leftSpeed << ", RightMotor: " << rightSpeed << endl;
#endif

	tcp->send(message, 5);
	tcp->send(speedMessage, 2);
}

void SpykeeManager::unplug()
{
	char message[] = { 80, 75, 16, 00, 1 };
	tcp->send(message, 5);
	message[0] = 5;
	tcp->send(message, 1);
}
