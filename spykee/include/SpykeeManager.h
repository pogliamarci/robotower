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
		char getCurrentMessageType();
		int getCurrentPayloadSize();
		void readDataFromSpykee();
		void sendMsg(uint8_t pkType, int length, const int8_t* payload);
		void authenticate(string username, string password);
		unsigned char buffer[SPYKEE_MAX_IMAGE];
		int bufferContentLength;
		bool hasNewImage;
		int lastBatteryLevel;
		bool batteryIsUnread;
	public:
		SpykeeManager(std::string username, std::string password) throw(SpykeeException);
		void setCameraStatus(bool setEnabled);
		void setLed(char ledid, bool status);
		inline bool hasBattery()
		{
			return batteryIsUnread;
		}
		inline int getBatteryLevel()
		{
			batteryIsUnread = false;
			return lastBatteryLevel;

		}
		std::vector<unsigned char>* readImage();
		void move(char leftSpeed, char rightSpeed);
		void unplug();
		inline bool hasImage()
		{
			return hasNewImage;
		}
		void readPacket();
};

#endif
