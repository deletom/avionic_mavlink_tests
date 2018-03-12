/**
 SERVER
 */
#include <stdio.h> /* Standard input/output definitions */
#include <stdlib.h>
#include <stdint.h> /* Standard types */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include "../lib/mavlink/standard/mavlink.h"

#define BUFFER_LENGTH 2041

int main(void) {

    struct termios toptions;
    int openSerial;
    int counterReceive = 0;

    openSerial = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

    if (openSerial == -1) {
        printf("init_serialport: Unable to open port ");
        //return -1;
    }

    if (tcgetattr(openSerial, &toptions) < 0) {
        printf("init_serialport: Couldn't get term attributes");
        //return -1;
    }

    speed_t brate = B57600;

    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    toptions.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    toptions.c_cc[VMIN] = 0;
    toptions.c_cc[VTIME] = 20;

    if (tcsetattr(openSerial, TCSANOW, &toptions) < 0) {
        printf("init_serialport: Couldn't set term attributes");
        // return -1;
    }

    while (1) {
        uint8_t buffer;
        mavlink_message_t message;
        mavlink_status_t status;
        int msgReceived;
        int msgReadyToDecode;

        msgReceived = read(openSerial, &buffer, 1);

        if (msgReceived > 0) {
            msgReadyToDecode = mavlink_parse_char(MAVLINK_COMM_1, buffer, &message, &status);
        }

        if (msgReadyToDecode) { 
            
            mavlink_highres_imu_t imu;
            mavlink_gps_raw_int_t position;            
            mavlink_heartbeat_t hearbeat;
            
            switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    mavlink_msg_heartbeat_decode(&message, &hearbeat);
                    printf("HEARTBEAT :  %d,%d \n", hearbeat.type, hearbeat.autopilot);
                    fflush(stdout);
                    break;

                case MAVLINK_MSG_ID_ATTITUDE:
                    mavlink_msg_highres_imu_decode(&message, &imu);
                    printf("IMU :  %f,%f,%f,%f \n", imu.xgyro, imu.ygyro, imu.zgyro,imu.pressure_alt);
                    fflush(stdout);
                    break;
                    
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                    mavlink_msg_gps_raw_int_decode(&message, &position);
                    printf("POSITION :  %d,%d \n", position.lat, position.lon);
                    fflush(stdout);                    
                    break;
                default:
                    printf("PB EQUIV : %d \n", message.msgid);
            }
            counterReceive = counterReceive +1;
        }
    }
    return EXIT_SUCCESS;
}


