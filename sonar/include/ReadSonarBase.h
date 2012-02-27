#ifndef READOSONAR_BASE_H_
#define READOSONAR_BASE_H_

#include <strings.h>  
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>  
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <sys/time.h>
#include <math.h>
#include <poll.h>
#include <exception>
#include <vector>
#include <string>
#include "SerialCommunication.h"
#include "CharCircularBuffer.h"

class ReadSonarDeviceException: public std::exception {
    public:
    virtual const char* what() const throw();
};

class ReadSonarBase{
    protected:
        unsigned int pack_n;

        bool ready;

        float to_meter;

        void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters );

        float * measure;
        CharCircularBuffer * buffer;
        char * tmp_buf;
    public:
        ReadSonarBase(float to_meter)
        throw (ReadSonarDeviceException);
        virtual ~ReadSonarBase();

        virtual unsigned int getLastPackNum();

        virtual float getMeasure(unsigned int index);

        virtual bool isReady()=0; //to implement

        virtual int readData()=0; //to implement

        virtual int sendRun()=0;//to implement
        virtual int sendStop()=0;//to implement

        virtual int parseLine();

        virtual unsigned int getLineToParseNum();

        virtual char * getParsedLine();

        static const unsigned int n_sonar = 12;

        static const int parse_err 		 = -1;

        static const int parse_meas_b1   =  0;
        static const int parse_meas_b2   =  1;
        static const int parse_meas_b3   =  2;
        static const int parse_meas_b4	 =  4;

        static const int parse_dbg		 =  3;
        static const int parse_ok		 =  4;
        static const int parse_response_err		=  5;

        static const int response_header_len = 4;
        static const char response_ok_header[];
        static const char response_err_header[];
        static const char response_dbg_header[];
        static const char response_meas_header[];

        static const int max_buf_tmp = 256;
};
#endif
