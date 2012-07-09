#ifndef SPYKEE_HEADER
#define SPYKEE_HEADER

#include "PracticalSocket.h"
#include <vector>

#define SPYKEE_MAX_IMAGE 10000

class SpykeeException: public exception
{
  virtual const char* what() const throw()
  {
    return "Error while communicating with the robot!";
  }
};

class SpykeeManager
{
	private:
		TCPSocket* tcp;
	public:

	SpykeeManager(char* username, char* password) throw(SpykeeException);

	void startCamera();

	std::vector<unsigned char>* getImage();

	~SpykeeManager()
	{}

	void move(int leftSpeed, int rightSpeed);

	void unplug();
};

#endif
