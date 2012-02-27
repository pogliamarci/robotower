#ifndef READOSONAR_H_
#define READOSONAR_H_

#include "ReadSonarBase.h"

class ReadSonar : public SerialCommunication, public ReadSonarBase{
        int fd;  	//descrittore del file per leggere/scrivere sulla seriale
        termios oldtio,newtio;

    public:
        int sendStringCommand(char *cmd,int len);
    public:
        ReadSonar(std::string sdev,float to_meter)
        throw (ReadSonarDeviceException);
        ~ReadSonar();

        virtual bool isReady();
        virtual int readData();
        virtual int sendRun();
        virtual int sendStop();
};

#endif
