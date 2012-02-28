#include "SpykeeManager.h"
#include "string.h"
#include "pthread.h"

typedef struct
{
	TCPSocket* sock;
	char buffer[300];
	int lenBuffer;
	bool free;
} structRecv;

using namespace std;

void HandleTCPClient(TCPSocket *sr);     // TCP client handling function
void *ThreadMain(void *arg);               // Main program of a thread 

SpykeeManager::SpykeeManager(char* user, char* pass)
{
    /* search a Spykee on the network, and tries to connect to it 
       with provided user and pass (default: user=admin pass=admin) */
	UDPSocket udp;
	char UDPMessage[] = {68, 83, 67, 86, 01}, respString[100];
	char message[] = {80, 75, 10, 00, 12};
	char messageAuthentication[strlen(user) + strlen(pass) + 2];
	string sourceAddress;
	unsigned short port;
	pthread_t threadID;
	structRecv sr;
	
	/* search a spykee... */
	udp.sendTo(UDPMessage, 5, "172.17.6.255", 9000);
	
	if (udp.recvFrom(respString, 100, sourceAddress, port) > 0)
	    printf("%s\n", (respString + 7));
	
	/* connects to it and authenticates */
	tcp = new TCPSocket(sourceAddress, 9000);
	
	tcp->send(message, 5);
	
	messageAuthentication[0] = 5;
	for (int i = 0; i < strlen(user); i++)
	{
		messageAuthentication[1 + i] = user[i];
	}
	messageAuthentication[1 + strlen(user)] = 5;
	for (int i = 0; i <  strlen(pass); i++)
	{
		messageAuthentication[2 + strlen(user) + i] = pass[i];
	}
	tcp->send(messageAuthentication, strlen(user) + strlen(pass) + 2);
	
	if (tcp->recv(respString, 100) > 0)
	{
		printf("%s\n", (respString + 7));
	}
	
	sr.sock = tcp;
	sr.lenBuffer = 0;
	sr.free = true;
	//Viene creato un thread per la ricezione dei messaggi
	//pthread_create(&threadID, NULL, ThreadMain, (void *) tcp);
	
	state |= Connected;
}

void SpykeeManager::unplug()
{
    /* sends command to make spykee unplug from charger */
	char message[] = {80, 75, 16, 00, 1};
	tcp->send(message, 5);
	message[0] = 5;
	tcp->send(message, 1);
}

void SpykeeManager::setplug()
{
    /* sends command to make spykee plug to charger */
	char message[] = {80, 75, 16, 00, 1};
	tcp->send(message, 5);
	message[0] = 5;
	tcp->send(message, 1);
}

void SpykeeManager::soundAllarm()
{
    /* make spykee execute an alarm-like sound */
	char message[] = {80, 75, 7, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	tcp->send(message, 5);
	if (tcp->recv(message, 15) > 0)
	{
		message[0] = 0;
		tcp->send(message, 1);
	}
}

void SpykeeManager::soundBomb()
{
    /* make spykee execute a bomb-like sound */
	char message[] = {80, 75, 7, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	tcp->send(message, 5);
	if (tcp->recv(message, 15) > 0)
	{
		message[0] = 1;
		tcp->send(message, 1);
	}
}

void SpykeeManager::turnOnCameraLight()
{
    /* make spykee turn on the light under the camera */
	printf("\nACCENSIONE!!");
	char message[] = {80, 75, 4, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	tcp->send(message, 5);
	//if (tcp->recv(message, 15) > 0)
	{
		message[0] = 0;
		message[1] = 1;
		tcp->send(message, 2);
		printf("\nACCESA!!!");
	}
}

void SpykeeManager::turnOffCameraLight()
{
    /* make spykee turn off the light under the camera */
	printf("\nSPEGNIMENTO!!");
	char message[] = {80, 75, 4, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	tcp->send(message, 5);
	//if (tcp->recv(message, 15) > 0)
	{
		message[0] = 0;
		message[1] = 0;
		tcp->send(message, 2);
		printf("\nSPENTA!!!");
	}
}

void SpykeeManager::move(int leftSpeed, int rightSpeed)
{
    /* make spykee move
     * speeds are in the range (-90, 90)
     * positive speeds move forward, negative ones backward
     */
	if ((leftSpeed != 0) && (rightSpeed != 0))
	{
		state |= Moving;
	}
	else
	{
		state &= ~Moving;
	}
	char message[] = {80, 75, 5, 0, 2};
	char speedMessage[] = {leftSpeed, rightSpeed, 0};
	//printf("\nLeftMotor: %d, RightMotor: %d", leftSpeed, rightSpeed);
	tcp->send(message, 5);
	tcp->send(speedMessage, 2);
}

void SpykeeManager::startCamera()
{
	char respString[100];
	int dimPackets = 0, bytesRecived;
	char firstMessage[] = {80, 75, 15, 0, 2};
	char secondMessage[] = {1, 1, 80, 75, 15, 0, 2, 2, 1};
	FILE* imm;
	
	tcp->send(firstMessage, 5);
	tcp->send(secondMessage, 9);
	
	state |= CameraOn;
}

int SpykeeManager::getState()
{
	return (enum states) state;
}

// TCP client handling function
void HandleTCPClient(TCPSocket *sock) {
	unsigned char buffer[1460];
	int recvMsgSize;
	while ((recvMsgSize = sock->recv(buffer, 1460)) > 0) {
		//printf("\nRICEVUTO MESSAGGIO");
		//fflush(stdin);
		SpykeeManager::pharseMessage(buffer, &recvMsgSize, NULL, SpykeeManager::pharseNewMessage);
		//printf("\nFINE ANALISI");
		//fflush(stdin);
	}
}

void *ThreadMain(void *clntSock) {
	// Guarantees that thread resources are deallocated upon return  
	pthread_detach(pthread_self()); 
	
	// Extract socket file descriptor from argument  
	HandleTCPClient((TCPSocket *) clntSock);
	
	//delete (TCPSocket *) clntSock;
	return NULL;
}

