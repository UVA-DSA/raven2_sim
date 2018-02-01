/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/**\file network_layer.cpp
 * \author Hawkeye King
 * \date 3/13/2004
 * \brief network thread
 *  read data out of fifo0 and put it on the network,
 *  listen to a network socket and put incoming data on fifo1,
 *  output a message every 1000 packets.
 * \version
 * ------------------------------------------
 * start
 * open socket on port 36000
 * listen on socket.
 * print whatever comes in over socket.
 * end
 * -------------------------------------------
*/


#include <sys/types.h>  // POSIX library: defines data types, provides FD_SET, FD_CLR, etc.
#include <sys/socket.h> // POSIX library: Internet Protocol family, provides socket constants
#include <sys/time.h>   // POSIX library: provides timers, time types and structures
#include <netinet/in.h> // POSIX library: defines socket ip protocols/address structs
#include <netdb.h>      // POSIX library: Definitions for network database operations, port/hostname lookup features.
#include <arpa/inet.h>  // POSIX library: Definitions for internet operations
#include <ctype.h>      // C Standard library: declares a set of functions to classify and transfrom individual characters
#include <errno.h>      // C Standard library: Defines macros to report error conditions
#include <stdio.h>      // C Standard library: Input and output operations
#include <fcntl.h>      // C Standard library: File control options
#include <time.h>       // C Standard library: timer, time types and structures
#include <ros/ros.h>    // Use ROS
#include <ros/console.h>// ROS console output header for ROS_DEBUG, unused

#include <stdlib.h>     // C Standard library: General Utilities Library
#include <string.h>     // C Standard library: String operations
#include <unistd.h>     // POSIX library: standard symbolic constants and types
//#include <rtai_fifos.h>

#include "itp_teleoperation.h"
#include "DS0.h"
#include "DS1.h"
#include "log.h"
#include "defines.h"


#ifdef packetgen
#define SERVER_PORT  "36001"             // used for packet_gen
#else
#define SERVER_PORT  "36000"             // used for gui
#endif
//#define SERVER_ADDR  "192.168.0.102"
//#define SERVER_ADDR  "128.95.205.206"    // used only if the robot needs to send data to the server
//#define SERVER_ADDR  "130.126.143.20"
#define SERVER_ADDR  "192.168.5.19"

#ifdef packetgen
extern int done_homing;
int first = 0;
extern struct device device0;
#define PACK_GEN_PORT  "32000"
#define RUN_PY_PORT  "34000"
#define HOST_ADDR  "127.0.0.1"    // used only if the robot needs to send data to the packet_gen
#endif

#ifdef save_logs
extern int logging;
#endif

#ifdef log_USB
#include <fstream>
extern std::ofstream NetworkPacketfile;
#endif

extern int receiveUserspace(void *u,int size);  // Defined in the local_io.cpp

/**\fn int initSock (const char* port )
  \brief This function initializes a socket
  \param port is a constant character pointer
  \return 0 if unitialized; non-negative integer(request_sock) if initialized sucessfully

  \ingroup Network
*/
int initSock (const char* port )
{
    int request_sock;
    struct servent *servp;       // stores port & protocol
    struct sockaddr_in server;   // socket address in AF_<family>

    //------------ init --------------//

    // create a socket descriptor, uninitialized.
    if ((request_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
        perror("socket");
        return 0;
    }

    // initialize port/protocol struct.
    if (isdigit(port[0]))
    {
        static struct servent s;
        servp = &s;
        s.s_port = htons((u_short)atoi(port));
    }
    else if ((servp = getservbyname(port,"tcp")) == 0)   // service lookup
    {
        perror("socket");
        return 0;
    }

    memset((void*) &server, 0, sizeof server);// initialize to null/all zeros?
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = servp->s_port;

    // "bind" the port to the socket.
    if (bind(request_sock, (struct sockaddr *) &server, sizeof server) < 0)
    {
        perror("bind");
        return 0;
    }

    //----------- end init -------------//



    return request_sock;
}


/**\fn int UDPChecksum(struct u_struct *u)
  \brief Calculate chesum for a teleoperation packet, not called anywhere
  \param u a u_struct pointer
  \struct u_struct structure passed from master to slave itp_teleoperation.h
  \return positive integer number
  \ingroup Network
*/
int UDPChecksum(struct u_struct *u)
{
    int chk=0;
    chk =  (u->surgeon_mode);
    chk += (u->delx[0])+(u->dely[0])+(u->delz[0]);
    chk += (u->delx[1])+(u->dely[1])+(u->delz[1]);
    chk += (u->buttonstate[0]);
    chk += (u->buttonstate[1]);
    chk += (int)(u->sequence);
    return chk;
}


// \todo DELET line 144-147? why volatile v_struct?
// Chek packet validity, incl. sequence numbering and checksumming
//int checkPacket(struct u_struct &u, int seq);
// main //
volatile struct v_struct v;


/**\fn void* network_process(void*)
  \brief This function receives and reads the udp package from the network in realtime, executed as an rt thread in rt_process_preempt.cpp
  \param param1 void pointer
  \return void
  \ingroup Network
*/
void* network_process(void* param1)
{
    int sock;              // sockets.
    int nfound, maxfd;
    const char *port = SERVER_PORT;
    fd_set rmask, mask;
    static struct timeval timeout = { 0, 500000 }; // .5 sec //
    struct u_struct u;

    int uSize=sizeof(struct u_struct);

    struct sockaddr_in clientName;
    int clientLength=sizeof(clientName);
    int retval;
    static int k = 0;
    int logFile;
    struct timeval tv;
    struct timezone tz;
    char logbuffer[100];
    unsigned int seq = 0;
    volatile int bytesread;

    // print some status messages
    log_msg("Starting network services..");
    log_msg("  u_struct size: %i",uSize);
    log_msg("  Using default port %s",port);

    ///// open log file
    logFile = open("err_network.log", O_WRONLY | O_CREAT | O_APPEND  | O_NONBLOCK , 0664);
    if ( logFile < 0 )
    {
        ROS_ERROR("ERROR: could not open log file.\n");
        exit(1);
    }
    gettimeofday(&tv,&tz);
    sprintf(logbuffer, "\n\nOpened log file at %s\n", ctime(&(tv.tv_sec)) );
    retval = write(logFile,logbuffer, strlen(logbuffer));

    /////  open socket
    sock = initSock(port);
    if ( sock <= 0)
    {
        ROS_ERROR("socket: service failed to initialize socket. (%d)\n",logFile);
        exit(1);
    }

    ///// setup sendto address
    memset(&clientName, 0, sizeof(clientName));
    clientName.sin_family = AF_INET;
    inet_aton(SERVER_ADDR, &clientName.sin_addr);
    clientName.sin_port = htons((u_short)atoi(port));

    clientLength=sizeof(clientName);

    ///// initialize data polling
    FD_ZERO(&mask);            // initialize a descriptor set fdset mask to the null set
    FD_SET(sock, &mask);       // add the descriptor sock in fdset mask
    maxfd=sock;

    log_msg("Network layer ready.");

    ///// Main read/write loop
    while ( ros::ok() )
    {
        rmask = mask;
        timeout.tv_sec = 2;  // hack:reset timer after timeout event.
        timeout.tv_usec = 0; //        ""

        // wait for i/o lines to change state //
        // Select() examines the I/O descriptor sets whose addresses are passed in fe_sets and returns the total number of ready descriptors in all the sets
        nfound = select(maxfd+1, &rmask, (fd_set *)0, (fd_set *)0, &timeout);
        // Select error
        if (nfound < 0)
        {
            if (errno == EINTR)
            {
                printf("interrupted system call\n");
                continue;
            }
            perror("select");
            break;
        }

// Select timeout: nothing to do
        if (nfound == 0)
        {
#ifdef packetgen
#ifdef packetgen_restart
            // To enable recieving packets again, set first
			if (device0.runlevel == 0)
				first = 0;
#endif
            if ((first == 0) && (device0.runlevel == 2))
			{
				done_homing = 0;
	            char v[6] = "Ready";
				clientName.sin_port = htons((u_short)atoi(PACK_GEN_PORT));
				inet_aton(HOST_ADDR, &clientName.sin_addr);
				sendto ( sock, (void*)&v, sizeof(v), 0,
				(struct sockaddr *) &clientName, clientLength);
#ifndef no_logging
                printf("=================> Sent READY to Packet Generator at ADDR = %d, PORT = %d\n", clientName.sin_addr, clientName.sin_port);
#endif
			}

			if ((first == 1) && (device0.runlevel == 2))
			{
    			char v[6] = "Done!";
				clientName.sin_port = htons((u_short)atoi(RUN_PY_PORT));
				inet_aton(HOST_ADDR, &clientName.sin_addr);
				sendto ( sock, (void*)&v, sizeof(v), 0,
				     (struct sockaddr *) &clientName, clientLength);
#ifndef no_logging
				printf("=================> Sent DONE to Run Injection at ADDR = %d, PORT = %d\n", clientName.sin_addr, clientName.sin_port);
#endif
#ifdef save_logs
				logging = 0;
#endif
			        //log_file("______________________________________________\n");
			        //logging = 0;
                                first = 2;
			}
#endif
            fflush(stdout);
            continue;
        }
#ifdef packetgen
        else {
            first = 1;
        }
#endif

        // Select: data on socket
        if (FD_ISSET( sock, &rmask))   // check whether the diescriptor sock is added to the fdset mask
        {
            bytesread = recvfrom(sock,
                                 &u,
                                 uSize,
                                 0,
                                 NULL,
                                 NULL);
#ifdef log_USB
        	for (int k = 0; k < uSize; k++)
        	{
        	    printf("%d ",(unsigned int)*((unsigned char*)(&u)+k));
        	    NetworkPacketfile << (unsigned int)*((unsigned char*)(&u)+k) << " ";
        	}
        	printf("\n");
        	NetworkPacketfile << "\n";
#endif

#ifdef mfi
//MFI_HOOK
//Conditions to check:
//If robot state is pedal down:
//if (device0.runlevel == 3)

//Start at packet 10 and continue for 100 packets:
//if ((u.sequence >= 10) && (u.sequence < 110))

//Variables to change (single or multiple, within or outside range):
//int u.delx[0], u.dely[0], u.delz[0] DONE(generate_u_delta_faults())
//float R_l[3][3] DONE(generate_u_R_l_faults())
//int surgeon_mode
//Data files to find the ranges for variables are inside: /home/raven/homa_wksp/Tests/
//Test_1, Test_2, Test_3 and others
// Result: t[1.194307:-1.269246], s[3.141593:-3.141593], p[3.141592:-3.141585]

#endif

#ifdef save_logs
      //log_msg("NETWORK) Receieved Data on Socket, Size = %d", uSize);
      //log_msg("Pos Arm 0 = %d, %d, %d", u.delx[0], u.dely[0], u.delz[0]);
      //log_msg("Pos Arm 1 = %d, %d, %d", u.delx[1], u.dely[1], u.delz[1]);


#endif
            if (bytesread != uSize){
                ROS_ERROR("ERROR: Rec'd wrong ustruct size on socket!\n");
                log_msg("%d not %d",bytesread, uSize);
                FD_CLR(sock, &rmask);   // remove the descriptor sock from fdset rmask
                continue;
            }

            if (k++ % 2000 == 0)
                log_msg(".");

//
//            if (u.checksum != UDPChecksum(&u))   // Check checksum
//            {
//                gettimeofday(&tv,&tz);
//                sprintf(logbuffer, "%s Bad Checksum -> rejected packet\n", ctime(&(tv.tv_sec)) );
//                ROS_ERROR("%s Bad Checksum -> rejected packet\n", ctime(&(tv.tv_sec)) );
//                retval = write(logFile,logbuffer, strlen(logbuffer));
//
//            }
//            else
            if (u.sequence == 0)        // Zero seqnum means reflect packet to sender
            {
                gettimeofday(&tv,&tz);
                sprintf(logbuffer, "%s Zero sequence -> reflect packet\n", ctime(&(tv.tv_sec)) );
                log_msg("%s Zero sequence -> reflect packet\n", ctime(&(tv.tv_sec)) );

                retval = write(logFile,logbuffer, strlen(logbuffer));

            }
            else if (u.sequence > seq+1)    // Skipping sequence number (dropped)
            {
                gettimeofday(&tv,&tz);
                sprintf(logbuffer, "%s Skipped (dropped?) packets %d - %d\n", ctime(&(tv.tv_sec)),seq+1, u.sequence-1 );
                ROS_ERROR("%s Skipped (dropped?) packets %d - %d\n", ctime(&(tv.tv_sec)),seq+1, u.sequence-1 );
                retval = write(logFile,logbuffer, strlen(logbuffer));
                seq = u.sequence;

                // TODO:: should this include a "receiveUserspace" call?

            }
            else if (u.sequence == seq)     // Repeated sequence number
            {
                gettimeofday(&tv,&tz);
                sprintf(logbuffer, "%s Duplicated packet %d - %d\n", ctime(&(tv.tv_sec)),seq, u.sequence );
                ROS_ERROR("%s Duplicated packet %d - %d\n", ctime(&(tv.tv_sec)),seq, u.sequence );
                retval = write(logFile,logbuffer, strlen(logbuffer));

            }

            else if (u.sequence > seq)       // Valid packet
            {
#ifdef save_logs
                //log_msg("NETWORK) Receieved Valid Packet # %d", u.sequence);
      		//log_msg("Pos Arm 0 = %d, %d, %d", u.delx[0], u.dely[0], u.delz[0]);
      		//log_msg("Pos Arm 1 = %d, %d, %d", u.delx[1], u.dely[1], u.delz[1]);
                //log_file("NETWORK) Receieved Valid Packet # %d\n", u.sequence);
#endif
                
// Remember 
		//log_msg("NETWORK) Receieved Valid Packet # %d\n", u.sequence);

		seq = u.sequence;
                receiveUserspace(&u,uSize);   // coordinates transform from ITP frame to robot 0 frame
            }


            // TODO: reset sequence should not be 'else if'  (maybe?)
            else if (seq > 1000 && u.sequence < seq-1000)       // reset sequence(skipped more than 1000 packets)
            {
                gettimeofday(&tv,&tz);
                sprintf(logbuffer, "%s Sequence numbering reset from %d to %d\n", ctime(&(tv.tv_sec)),seq, u.sequence );
                log_msg("%s Sequence numbering reset from %d to %d\n", ctime(&(tv.tv_sec)),seq, u.sequence );
                seq = u.sequence;
                retval = write(logFile,logbuffer, strlen(logbuffer));
            }
            else
            {
                gettimeofday(&tv,&tz);
                sprintf(logbuffer, "%s Out of sequence packet %d\n", ctime(&(tv.tv_sec)),seq );
                ROS_ERROR("%s Out of sequence packet %d\n", ctime(&(tv.tv_sec)),seq );
                retval = write(logFile,logbuffer, strlen(logbuffer));
            }
        }
// code i need
#ifdef packetgen_restart
	if (device0.runlevel == 0)
	{
		char v[8] = "Stopped";
		clientName.sin_port = htons((u_short)atoi(PACK_GEN_PORT));
		inet_aton(HOST_ADDR, &clientName.sin_addr);
		sendto ( sock, (void*)&v, sizeof(v), 0,
		(struct sockaddr *) &clientName, clientLength);
		printf("=================> Sent Stopped to Packet Generator at ADDR = %d, PORT = %d\n", clientName.sin_addr, clientName.sin_port);
#ifdef dyn_simulator
		clientName.sin_port = htons((u_short)atoi(RUN_PY_PORT));
		inet_aton(HOST_ADDR, &clientName.sin_addr);
		sendto ( sock, (void*)&v, sizeof(v), 0,
		(struct sockaddr *) &clientName, clientLength);
		printf("=================> Sent E-Stopped to Run Injection at ADDR = %d, PORT = %d\n", clientName.sin_addr, clientName.sin_port);
#endif
	}
#endif
#ifdef NET_SEND
        sendto ( sock, (void*)&v, vSize, 0,
                 (struct sockaddr *) &clientName, clientLength);
#endif

    } // end while(ros::ok())

    close(sock);

    log_msg("Network socket is shutdown.");
    return(NULL);
} // main - server.c //

/* int checkPacket(struct u_struct &u, int seq) */
/* { */
/*   struct timeval tv; */
/*   struct timezone tz; */

/*   if (u.checksum != UDPChecksum(&u) ) { // Check checksum */
/*     gettimeofday(&tv,&tz); */
/*     sprintf(logbuffer, "%s Bad Checksum -> rejected packet\n", ctime(&(tv.tv_sec)) ); */
/*     printf("%s Bad Checksum -> rejected packet\n", ctime(&(tv.tv_sec)) ); */
/*     write(logFile,logbuffer, strlen(logbuffer)); */

/*   } else if (u.sequence == 0) {      // Zero seqnum means reflect packet to sender */
/*     gettimeofday(&tv,&tz); */
/*     sprintf(logbuffer, "%s Zero sequence -> reflect packet\n", ctime(&(tv.tv_sec)) ); */
/*     printf("%s Zero sequence -> reflect packet\n", ctime(&(tv.tv_sec)) ); */

/*     write(logFile,logbuffer, strlen(logbuffer)); */

/*   } else if (u.sequence > seq+1){   // Skipping sequence number (dropped) */
/*     gettimeofday(&tv,&tz); */
/*     sprintf(logbuffer, "%s Skipped (dropped?) packets %d - %d\n", ctime(&(tv.tv_sec)),seq+1, u.sequence-1 ); */
/*     printf("%s Skipped (dropped?) packets %d - %d\n", ctime(&(tv.tv_sec)),seq+1, u.sequence-1 ); */
/*     write(logFile,logbuffer, strlen(logbuffer)); */
/*     seq = u.sequence; */

/*   } else if (u.sequence == seq){    // Repeated sequence number */
/*     gettimeofday(&tv,&tz); */
/*     sprintf(logbuffer, "%s Duplicated packet %d - %d\n", ctime(&(tv.tv_sec)),seq ); */
/*     printf("%s Duplicated packet %d - %d\n", ctime(&(tv.tv_sec)),seq ); */
/*     write(logFile,logbuffer, strlen(logbuffer)); */

/*   } else if (u.sequence > seq){      // Valid packet */
/*     seq = u.sequence; */
/*     retval=write(fifo1, &u, uSize); // Write data to module via FIFO 1. */

/*     // Print some diagnostics. */
/*     // printf("%d\t%d\t%d\t\n", u.delx[0], u.dely[0], u.delz[0]); */
/*     if (retval != uSize) */
/*       printf("error writing fifo 1: expected %d rec'd %d\n", uSize, retval); */

/*   } else { */
/*     gettimeofday(&tv,&tz); */
/*     sprintf(logbuffer, "%s Out of sequence packet %d\n", ctime(&(tv.tv_sec)),seq ); */
/*     printf("%s Out of sequence packet %d\n", ctime(&(tv.tv_sec)),seq ); */
/*     write(logFile,logbuffer, strlen(logbuffer)); */

/*   } */

/* }  */
