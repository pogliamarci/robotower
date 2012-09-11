#ifndef SPYKEE_HEADER
#define SPYKEE_HEADER

#include "PracticalSocket.h"
#include <vector>
#include <string>
#include <stdint.h>

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
		char getMessageType();
		int getPayloadSize();
		void readDataInBuffer();
		void sendMsg(uint8_t pkType, int length, const int8_t* payload);
		void authenticate(string username, string password);
		unsigned char buffer[SPYKEE_MAX_IMAGE];
		int bufferContentLength;
		bool hasNewImage;
	public:
		SpykeeManager(std::string username, std::string password) throw(SpykeeException);
		void setCameraStatus(bool setEnabled);
		void setLed(char ledid, bool status);
		std::vector<unsigned char>* getImage();
		void move(char leftSpeed, char rightSpeed);
		void unplug();
		inline bool hasImage()
		{
			return hasNewImage;
		}
		void readPacket();
};

#endif
