#include "ReadSonar.h"
#include <iostream>
#include "ros/ros.h"
#include "sonar/Sonar.h"
#include "sonar/Led.h"

#define SERIAL_DEVICE_FILENAME "/dev/ttyUSB0"

#define NORTH 			0
#define SOUTH			1
#define NORTH_EAST		2

#define EAST			6
#define WEST			7
#define NORTH_WEST		8

using namespace std;

ReadSonar* initializeSonar();
void doAction(ReadSonar*, ros::Publisher*);
void readDataFromSonar(ReadSonar*);

int main(int argc, char** argv) {
    try {
        ReadSonar* readSonar = initializeSonar();
    
        ros::init(argc, argv, "sonar");
        ros::NodeHandle n;
    
        /* initialization ase publisher of sonar_data msgs */
        ros::Publisher sonar_data_pub = n.advertise<sonar::Sonar>("sonar_data", 1000);
        
        doAction(readSonar, &sonar_data_pub);
    
        delete readSonar;
        return 0;
    } catch  ( ReadSonarDeviceException &e ){
        cerr << "Read sonar device exception\n";
        cerr << e.what() << "\n";
    }
    return 1;
}

ReadSonar* initializeSonar() {
    ReadSonar* readSonar = new ReadSonar( SERIAL_DEVICE_FILENAME, 1 );
    if ( readSonar ) {
        readSonar->sendRun();
        char c = (char) 0;
        readSonar->sendStringCommand(&c,1);
        sleep(3);
    }
    return readSonar;
}

void doAction(ReadSonar* readSonar, ros::Publisher* sonar_data_pub_ptr) {  
    
    ros::Publisher sonar_data_pub = *sonar_data_pub_ptr;
    while(ros::ok()){
        readDataFromSonar(readSonar);
        /* build the message and publish it */
        sonar::Sonar* msg = new sonar::Sonar();
        msg->north = readSonar->getMeasure(NORTH);
        msg->south = readSonar->getMeasure(SOUTH);
        msg->east = readSonar->getMeasure(EAST);
        msg->west = readSonar->getMeasure(WEST);
        sonar_data_pub.publish(*msg);
        ros::spinOnce();
        delete msg;
    }

/* LED...
    char c;

    c = (char) 0x00;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x01;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x02;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x03;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x04;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x05;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x08;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x09;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x0A;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0xB;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0xC;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x00;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x10;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x20;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0x40;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
    c = (char) 0xFF;
    readSonar->sendStringCommand(&c,1);
    sleep(3);
*/

    /*
     *
     * CODICE PER GESTIRE I LED....
     *
     *
	unsigned char *buffer = NULL;
	char * value = (char*)malloc(sizeof(char)*100);

	energy = false;

	if ( ReceiveLastMessage( MSG_TO_SONAR_FROM_GESTURE, buffer, false ) > 0 )
	{
		int dim;
		dim = FindTokenValue((char*)buffer, "<ENERGY>", value);

		// Se arriva il tag energy ad 1, indica che si deve togliere un punto di energia

		if(strncmp(value, "0",1) == 0)
		{
			energyLeds =0;
		}

		else if(strncmp(value, "1",1) == 0)
		{
			energyLeds =1;
		}

		else if(strncmp(value, "2",1) == 0)
		{
			energyLeds =2;
		}

		else if(strncmp(value, "3",1) == 0)
		{
			energyLeds =3;
		}

		else if(strncmp(value, "4",1) == 0)
		{
			energyLeds =4;
		}


		if (energyLeds<=0)
		{
			energyLeds = 0;

			cout<<"SONO MORTOOOOOOOOOOOOOOOO :("<<endl;
			exit(0);
		}


		dim = FindTokenValue((char*)buffer, "<LOCK>", value);
		if(strncmp(value, "8",1) == 0)
		{
			energyLeds |= 8;
		}
	}
    */
}

void readDataFromSonar(ReadSonar* readSonar) {
    /* ma a che serve sta roba?!?!?!??! */
    static int meas_progress=0;
    static int line_readed=0;
    if(readSonar->readData()==0){
        unsigned int n_line;
        n_line=readSonar->getLineToParseNum();
        line_readed+=n_line;

        for(unsigned int i=0;i<n_line;i++){
            switch(readSonar->parseLine()){
            case ReadSonar::parse_err:
                cerr << "Error in parse method :" << readSonar->getParsedLine() << endl;
                cout << "NORD: " << readSonar->getMeasure(NORTH) << endl;
                cout << "SUD: "  << readSonar->getMeasure(SOUTH) << endl;
                meas_progress=0;
                break;
            case ReadSonar::parse_meas_b1:
                //FileLogger::write("found meas group 1:");
                //FileLogger::writeLine(readSonar->getParsedLine());
                if(meas_progress==0)meas_progress++;
                break;
            case ReadSonar::parse_meas_b2:
                //FileLogger::write("found meas group 2:");
                //FileLogger::writeLine(readSonar->getParsedLine());
                if(meas_progress==1)meas_progress++;
                break;
            case ReadSonar::parse_meas_b3:
                //FileLogger::write("found meas group 3:");
                //FileLogger::writeLine(readSonar->getParsedLine());
                if(meas_progress==2)meas_progress++;
                break;
            case ReadSonar::parse_meas_b4:
                // FileLogger::write("found meas group 4:");
                // FileLogger::writeLine(readSonar->getParsedLine());
                if(meas_progress==3)meas_progress++;
                break;
            case ReadSonar::parse_dbg:
                printf("found DBG line <%s>",readSonar->getParsedLine());
                meas_progress=0;
                break;
                //                  case ReadSonar::parse_ok:
                //                      printf("found OK line <%s>",readSonar->getParsedLine());
                //                      meas_progress=0;
                //                      break;
            case ReadSonar::parse_response_err:
                printf("found ERR line <%s>",readSonar->getParsedLine());
                meas_progress=0;
                break;
            default:
                printf("defensive programming... unrechable... mumble mumble\n");
                meas_progress=0;
                break;
            }
        }
    }
}
