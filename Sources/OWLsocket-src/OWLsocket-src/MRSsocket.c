// Version 2 Corrected Order of Data from Left Right Neck to Right Left Neck

/* A socket Server based on myServer.c (see below for reference)
and a translation of PYTHON code from PFCsocket.py
to drive the OWL-BOTS eyes
M R Simpson 25th May 2017
NB the following!
This will open a TCP Socket using (ip:port) 10.0.0.10:12345
Expect a packet of 5 Integers Space delimited (as a string)
Each Integer will have a capped range of 1200 to 2000 (inclusive)
Number of Integers * Size of Integers + Space minus one space as
Packet terminated by CR/LF thus: 5 * (4 + 1) - 1 = 24 (Size of packet array

compiled using the following command line (For reference)
pi@raspberry:~ $ gcc -Wall -pthread -o MRSsocket MRSsocket.c -lpigpio_if2

and to execute code
pi@raspberry:~ $ ./MRSsocket
*/

/* A simple server in the internet domain using TCP. myServer.c
D. Thiebaut
Adapted from http://www.cs.rpi.edu/~moorthy/Courses/os98/Pgms/socket.html
The port number used in 51717.
This code is compiled and run on the Raspberry as follows:

    g++ -o myServer myServer.c
    ./myServer

The server waits for a connection request from a client.
The server assumes the client will send positive integers, which it sends back multiplied by 2.
If the server receives -1 it closes the socket with the client.
If the server receives -2, it exits.
*/

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <signal.h>
#include <pigpiod_if2.h>

#define NUM_GPIO 32
// Minimum and Maxium Values taken from PFCsocket.py
#define MIN_WIDTH_LX 1180
#define MAX_WIDTH_LX 1850

#define MIN_WIDTH_LY 1180
#define MAX_WIDTH_LY 2000

#define MIN_WIDTH_RX 1200
#define MAX_WIDTH_RX 1890

#define MIN_WIDTH_RY 1120
#define MAX_WIDTH_RY 2000

#define MIN_WIDTH_NECK 1100
#define MAX_WIDTH_NECK 1950

//For Reference IP is   10.0.0.10
#define MYPORT          12345

#define LEFT_X_PIN      14
#define LEFT_Y_PIN      16
#define RIGHT_X_PIN     15
#define RIGHT_Y_PIN     17
#define NECK_PIN        13

int run = 1;

int freq_LX, freq_LY, freq_RX, freq_RY, freq_NECK;

void stop(int signum)
{
        run = 0;
}

//Left in for future use??? Not used in this application no speed loading
void sendData( int sockfd, int x ) {
        int n;
        char buffer[32];
        sprintf( buffer, "%d\n\r", x );
        if ( (n = write( sockfd, buffer, strlen(buffer) ) ) < 0 )
                printf("ERROR writing to socket\n\r");
        buffer[n] = '\0';
}

int getData( int sockfd ) {
        char buffer[32];
        int n;
        //Read data from socket place into buffer[] and truncate to 32 Bytes
        //return in n number of byets truncated at
        if ( (n = read(sockfd,buffer,32) ) < 0 )
                printf("ERROR reading from socket\n\r");
        //Add a NULL character at nth position to ensure we have a String type in Buffer
        //In case of variable size differences and previously stored data in buffer[] memory
        buffer[n] = '\0';
        //Left for debugging but can be removed/commented to speed up process as outputs to stdio console
        //                 printf("String received is %s\n\r",buffer);
        //Scan String pointed to by buffer and extract Integers from buffer[] string
        sscanf(buffer,"%d %d %d %d %d", &freq_RX, &freq_RY, &freq_LX, &freq_LY, &freq_NECK);
        //Left for debugging but can be removed/commented to speed up process as outputs to stdio console
        //                 printf("%d %d %d %d %d\n\r", freq_LX, freq_LY, freq_RX, freq_RY, freq_NECK);

        return atoi( buffer );
}

// Type defined for function "setSignalHandler"
typedef void (*signalFunc_t) (int signum);

static void setSignalHandler(int signum, signalFunc_t sigHandler)
{
        struct sigaction new;
        memset(&new, 0, sizeof(new));
        new.sa_handler = sigHandler;
        sigaction(signum, &new, NULL);
}

int main(int argc, char *argv[])
{
        int sockfd, newsockfd, portno = MYPORT, clilen;
        struct sockaddr_in serv_addr, cli_addr;
        int data;
        int pi;

        pi = pigpio_start(NULL, NULL);          //Starts pigpio and sets handler to name "pi"
        printf("and done\n\r");
        if ( pi < 0 )
        {
                printf("Error Code %d exiting program\n\r", pi);
                return -1;
        }

        setSignalHandler(SIGINT, stop);         // What to do in the event of a CTRL-c from console, function stop() sets run to 0

        printf( "using port #%d\n\r", portno ); // Confirmation of port used to console

        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0)  printf("ERROR opening socket\n\r");
        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons( portno );
        if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) printf("ERROR on binding\n\r");
        listen(sockfd,5);
        clilen = sizeof(cli_addr);

        // Now wait for a connection (Use PuTTY raw settings IP 10.0.0.10 and port 12345 N.B. see Defines above to confirm.
        while ( run )
        {
        printf( "waiting for new client...\n\r" );
        if ( ( newsockfd = accept( sockfd, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen) ) < 0 )
                printf("ERROR on accept\n\r");
        printf( "opened new communication with client\n\r" );
        while ( run )
        {
                // wait for the data to come in the designated Socket
                data = getData( newsockfd );
                // in this Order RIGHT X , RIGHT Y , LEFT X , LEFT Y, NECK
                // can remove the following line to speed up by not outputing values to console, N.B. left in for debugging perposes
                //     printf( "got %d %d %d %d %d ", freq_LX, freq_LY , freq_RX, freq_RY, freq_NECK );
                // WARNING! Limit parameters BEFORE using to set servos to prevent Damage; change at your own peril !!!!
                if (freq_LX<MIN_WIDTH_LX) freq_LX=MIN_WIDTH_LX;
                if (freq_LX>MAX_WIDTH_LX) freq_LX=MAX_WIDTH_LX;

                if (freq_LY<MIN_WIDTH_LY) freq_LY=MIN_WIDTH_LY;
                if (freq_LY>MAX_WIDTH_LY) freq_LY=MAX_WIDTH_LY;

                if (freq_RX<MIN_WIDTH_RX) freq_RX=MIN_WIDTH_RX;
                if (freq_RX>MAX_WIDTH_RX) freq_RX=MAX_WIDTH_RX;

                if (freq_RY<MIN_WIDTH_RY) freq_RY=MIN_WIDTH_RY;
                if (freq_RY>MAX_WIDTH_RY) freq_RY=MAX_WIDTH_RY;

                if (freq_NECK<MIN_WIDTH_NECK) freq_NECK=MIN_WIDTH_NECK;
                if (freq_NECK>MAX_WIDTH_NECK) freq_NECK=MAX_WIDTH_NECK;

                printf( "using %d %d %d %d %d \n\r", freq_LX, freq_LY , freq_RX, freq_RY, freq_NECK );

                if ( data < 0 )  break;

                // drive the servo(s) here
                set_servo_pulsewidth(pi,LEFT_X_PIN,freq_LX);
                set_servo_pulsewidth(pi,LEFT_Y_PIN,freq_LY);
                set_servo_pulsewidth(pi,RIGHT_X_PIN,freq_RX);
                set_servo_pulsewidth(pi,RIGHT_Y_PIN,freq_RY);
                set_servo_pulsewidth(pi,NECK_PIN,freq_NECK);
                //Don't need to do the following - just slows process down left for debugging
                //--- send new data back ---
                //printf( "sending back %d\n\r", data );
                sendData( newsockfd, data );
                }
                close( newsockfd );
                //--- if -2 sent by client, we can quit ---
                if ( data == -2 )
                {
                        break;
                }
        }
        return 0;
}
