#ifndef SPYKEE_HEADER
#define SPYKEE_HEADER

#include "PracticalSocket.h"
#include <vector>
#include <string>

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
		bool containsNewImage(unsigned char buffer[]);
		int getImageSize(unsigned char buffer[]);

	public:
		SpykeeManager(std::string username, std::string password) throw(SpykeeException);
		void startCamera();
		std::vector<unsigned char>* getImage();
		void move(char leftSpeed, char rightSpeed);
		void unplug();
};

#endif
