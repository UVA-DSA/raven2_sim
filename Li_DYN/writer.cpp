#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include "cmath"

#define MAX_BUF 1024

using namespace std;

int main()
{
    int fd1,fd2;
    char buf[MAX_BUF];
    int arm_type = 0;
    int packet_num = 0;
    string line;
    char wrfifo[20] = "/tmp/dac_fifo";
    char rdfifo[20] = "/tmp/mpos_vel_fifo";
    char outfile[20] = "./output.csv";
    /* create the FIFO (named pipe) */
    mkfifo(wrfifo, 0666);
    printf("Write FIFO Created\n");
    /* write "Hi" to the FIFO */
    fd1 = open(wrfifo, O_WRONLY);
	printf("Write FIFO Opened\n");
	fd2 = open(rdfifo, O_RDONLY);
    printf("Read FIFO Opened\n");
    FILE *f = fopen(outfile, "w");
    double mpos[4], mvel[4], dac[4], est_mpos[4], est_mvel[4], est_jpos[4];
    //char buff[50] = "pos_des.txt";
    char buff[50] = "dac_mvel_mpos.txt";
    std::ifstream datafile;
    datafile.open(buff,std::ifstream::in);
    if (datafile.is_open())
    {
		getline(datafile,line);
        int i=0;
        while (getline(datafile,line))
		//for (int j =0; j < 5;j++)
        {
			//getline(datafile,line);
			// Read data from file
			istringstream iss(line);
            iss >>  mpos[0] >> mpos[1] >> mpos[2] >>
					mvel[0] >> mvel[1] >> mvel[2] >>
					dac[0] >> dac[1] >> dac[2];
			packet_num = packet_num + 1;
            // Send simulator input to FIFO
			sprintf(buf, "%d %d %f %f %f %f %f %f %f %f %f", arm_type, packet_num,
		           mpos[0]*M_PI/180,mpos[1]*M_PI/180,mpos[2]*M_PI/180,
	 			   mvel[0]*M_PI/180,mvel[1]*M_PI/180,mvel[2]*M_PI/180,
                   -dac[0], -dac[1], -dac[2]);
    	    write(fd1, buf, sizeof(buf));
 	        printf("\nSent motor velocities and DACs:\n%f,%f,%f,\n%f,%f,%f,\n%f,%f,%f\n",
				   mpos[0]*M_PI/180,mpos[1]*M_PI/180,mpos[2]*M_PI/180,
	 			   mvel[0]*M_PI/180,mvel[1]*M_PI/180,mvel[2]*M_PI/180,
                   dac[0], dac[1], dac[2]);
			// Read estimates from FIFO
 		    read(fd2, buf, sizeof(buf));
			// Write the results to the screen
            stringstream ss(buf);
            ss >> est_mpos[0] >> est_mvel[0] >> est_jpos[0] >>
            	  est_mpos[1]  >> est_mvel[1] >> est_jpos[1] >>
				  est_mpos[2] >> est_mvel[2] >> est_jpos[2];
            printf("Received estimated motor positions/velocties and joint positions:\n(%f, %f, %f),\n (%f, %f, %f),\n (%f, %f, %f),\n",est_mpos[0], est_mvel[0], est_jpos[0],est_mpos[1], est_mvel[1],est_jpos[1],est_mpos[2], est_mvel[2],est_jpos[2]);
            fprintf(f,"%f,%f,%f,%f,%f,%f\n",est_mpos[0], est_mvel[0], est_mpos[1], est_mvel[1],est_mpos[2], est_mvel[2]);
		}
	}
    /* remove the FIFO */
    unlink(wrfifo);
	close(fd1);
	close(fd2);
    fclose(f);
    return 0;
}
