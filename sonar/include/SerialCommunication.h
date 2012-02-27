#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <poll.h>
#include <stdio.h>
#include <errno.h>

class SerialCommunication{
        struct pollfd ufd[1];

    public:
        static const int wait_ok=1;
        static const int wait_tout=0;
        static const int wait_err=-1;

        void set_fd(int fd);

        int waitData(int msec_tout);
        //int set_low_latency();
};


#endif
