#include "SpykeeManager.h"
#include <vector>
#include <iostream>
#include <stdio.h>
#include <string.h>

#define DEBUG_SPYKEE

using namespace std;

SpykeeManager::SpykeeManager(char* username, char* password)
{
	//definisco i nomi utente e password

#ifdef DEBUG_SPYKEE
	cout << "Nome utente: " << username << endl;
	cout << "Password: " << password << endl;
#endif

	//inizializzo i campi per la connessione
	UDPSocket udp;
	char UDPMessage[] = { 68, 83, 67, 86, 01 }, respString[100];
	char message[] = { 80, 75, 10, 00, 12 };
	char messageAuthentication[strlen(username) + strlen(password) + 2];
	string sourceAddress;
	unsigned short port;
	structRecv sr;

	//avvio la connessione
	udp.sendTo(UDPMessage, 5, "172.17.6.255", 9000);

	if (udp.recvFrom(respString, 100, sourceAddress, port) > 0)
		printf("%s\n", (respString + 7));

	tcp = new TCPSocket(sourceAddress, 9000);

	tcp->send(message, 5);

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
	tcp->send(messageAuthentication, strlen(username) + strlen(password) + 2);
	if (tcp->recv(respString, 100) > 0)
	{
		printf("%s\n", (respString + 7));
	}
	sr.sock = tcp;
	sr.lenBuffer = 0;
	sr.free = true;
#ifdef DEBUG_SPYKEE
	cout << "Connessione effettuata" << endl;
#endif
}

void SpykeeManager::startCamera()
{
	//dico a Spykee di inizializzare la telecamera
	char firstMessage[] = { 80, 75, 15, 0, 2 };
	char secondMessage[] = { 1, 1, 80, 75, 15, 0, 2, 2, 1 };
	tcp->send(firstMessage, 5);
	tcp->send(secondMessage, 9);

#ifdef DEBUG_SPYKEE
	cout << "Camera accesa" << endl;
#endif
}

/* returns an Image objects dynamically allocated, containing the image data */
vector<unsigned char>* SpykeeManager::getImage()
{
	//inizializzo le variabili per leggere le immagini
	unsigned char buffer[SPYKEE_MAX_IMAGE];
	int recvMsgSize;
	enum stati
	{
		nonTrovata, acquisizione, immagineFinita
	};
	int stato = nonTrovata;

	/* static unsigned char immagine[SPYKEE_MAX_IMAGE]; */

	vector<unsigned char>* image_data = new vector<unsigned char>(SPYKEE_MAX_IMAGE);

	/* unsigned int lengthImage; */

	unsigned int posizioneCorrente;
	unsigned int image_length;

	/*// no more needed...
	FILE* file;
	*/

	//inizio ciclo acquisizione
	while (!(stato == immagineFinita))
	{
		recvMsgSize = tcp->recv(buffer, SPYKEE_MAX_IMAGE);
		#ifdef DEBUG_SPYKEE
		cout << endl << endl << endl << endl << "Messaggio arrivato di " << recvMsgSize << " byte." << endl;
		#endif

		if (stato == acquisizione)
		{
			#ifdef DEBUG_SPYKEE
			cout << "Continuazione dell'acquisizione" << endl;
			cout << "La posizione corrente di riempimento del buffer è partita da " << posizioneCorrente;
			#endif

			for (register int i = 0;
					((i < recvMsgSize) && (posizioneCorrente < image_length));
					i++)
			{
				image_data->at(posizioneCorrente) = buffer[i];

				if ((i < recvMsgSize) && (posizioneCorrente < image_length))
					posizioneCorrente++;
			}

			#ifdef DEBUG_SPYKEE
			cout << " ed è finita a " << posizioneCorrente << endl;
			#endif

			if (posizioneCorrente >= image_length)
			{
				stato = immagineFinita;
			}
			else
			{
				stato = acquisizione;
			}

			#ifdef DEBUG_SPYKEE
			cout <<"Ci sono ancora: " << (image_length - posizioneCorrente) << " byte"<< endl;
			cout <<"Ricordo che la dimensione totale dell'immagine è di " << image_length << " byte" << endl;
			#endif
		}
		//controllo se è arrivata una nuova immagine
		if (((buffer[0] == 80) && (buffer[1] == 75) && (buffer[2] == 2))
				&& (stato == nonTrovata))
		{
			#ifdef DEBUG_SPYKEE
			cout << "Il messaggio contine una nuova immagine" << endl;
			#endif

			//ottengo la lunghezza dell'immagine
			image_length = (buffer[3] << 8) + buffer[4];

			#ifdef DEBUG_SPYKEE
			cout << "L'immagine contenuta nel pacchetto è lunga " << image_length << " byte" << endl;
			#endif

			register int i;

			//scrivo l'immagine ricevuta nella mia in locale
			for (i = 5, posizioneCorrente = 0;
					((i < recvMsgSize) && (posizioneCorrente < image_length));
					i++, posizioneCorrente++)
			{
				image_data->at(posizioneCorrente) = buffer[i];
			}

			#ifdef DEBUG_SPYKEE
			cout << "La posizione corrente di riempimento del buffer è partita da 0 ed è finita a " << posizioneCorrente << endl;
			#endif

			if (posizioneCorrente < image_length)
			{
				stato = acquisizione;
			}
			else
			{
				stato = immagineFinita;
			}
		}
		#ifdef DEBUG_SPYKEE
		cout << "Il pacchetto è stato  processato"<<endl;
		#endif
	}

	//fine acquisizione immagine
	#ifdef DEBUG_SPYKEE
	cout << "Immagine acquisita";
	#endif
	/*
	file = fopen("image.jpg", "wb");
	fwrite(immagine, 1, posizioneCorrente + 1, file);
	fclose(file);*/
	return image_data;
}

void SpykeeManager::move(int leftSpeed, int rightSpeed)
{
	char message[] = { 80, 75, 5, 0, 2 };
	char speedMessage[] = { leftSpeed, rightSpeed, 0 };

#ifdef DEBUG_SPYKEE
	printf("\nLeftMotor: %d, RightMotor: %d", leftSpeed, rightSpeed);
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
