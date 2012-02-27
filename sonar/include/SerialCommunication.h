/*
 * RoboTower, Hi-CoRG based on ROS
 *
 *
 * Copyright (C) 2011 Marcello Pogliani, Davide Tateo
 * Versione 1.0
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

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
