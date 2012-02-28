#ifndef SPYKEE_HEADER
#define SPYKEE_HEADER

#include "PracticalSocket.h"
#include <vector>

#define SPYKEE_MAX_IMAGE 10000

typedef struct
	{
		TCPSocket* sock;
		char buffer[300];
		int lenBuffer;
		bool free;
	} structRecv;

class SpykeeManager
{
	private:
		TCPSocket* tcp;
	public:

	SpykeeManager(char* username, char* password);

	void startCamera();

	std::vector<unsigned char>* getImage();

	~SpykeeManager()
	{}

	void move(int leftSpeed, int rightSpeed);

	void unplug();
};

#endif
