#include "two_arm_dyn.h"
#include <time.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>

#define MAX_BUF 1024


//#define LOGGING
#define EULER_INT
//#define UPDATE_MODEL
//#define UPDATE_FREQ 20
//Integration steps in msec
double RK_STEP = 0.001;
double EULER_STEP = 0.001;

int iter_time_gold, iter_time_green=0;
double DACs[3];

int main()
{
	struct timespec t1, t2;
	clock_t t_start,t_end;
    double mpos[3],mvel[3],jpos,jvel;
    int arm_type, packet_num;
	double sum_d = 0.0;
	int fd1,fd2;
    char rdfifo[20] = "/tmp/dac_fifo";
    char wrfifo[20] = "/tmp/mpos_vel_fifo";
    mkfifo(wrfifo, 0667);
    //printf("Write FIFO Created..\n");
	char buf[MAX_BUF];
    /* open, read, and display the message from the FIFO */
    fd1 = open(rdfifo, O_RDONLY);
    //printf("Read FIFO Opened..\n");
    fd2 = open(wrfifo, O_WRONLY);
    //printf("Write FIFO Opened..\n");
	// Robot state
    state_type r_state = {0,0,0,0,0,0,0,0,0,0,0,0};

	while(read(fd1, buf, MAX_BUF))
	//while (iter_time_gold < 100)
	{
		//read(fd1, buf, MAX_BUF);
        stringstream ss(buf);
        ss >> arm_type >> packet_num >>
			  mpos[0]>>mpos[1]>>mpos[2]>>mvel[0]>>mvel[1]>>mvel[2]>>DACs[0]>>DACs[1]>>DACs[2];
		//cout << "\n########## Packet #"<<packet_num<<" ##########\n";
        //printf("Received motor velocities and dacs for %s arm:\n%f, %f, %f\n%f, %f, %f\n%d, %d, %d\n", (arm_type==1)?"Green":"Gold",mpos[0],mpos[1],mpos[2],mvel[0],mvel[1],mvel[2],int(DACs[0]),int(DACs[1]),int(DACs[2]));

	    if (arm_type == 0)
		{
			// initial state
#ifdef UPDATE_MODEL
		    if (iter_time_gold % UPDATE_FREQ == 0)
#else
		    if (iter_time_gold == 0)
#endif
			{
				for (int i = 0; i < 3; i++)
				{
					switch (i)
					{
						case 0:
							jpos = 0.007342345766264*mpos[0]-PI;
							jvel = 0.007342345766264*mvel[0];
						break;
						case 1:
							jpos = 0*-0.001067944191703*mpos[0]+0.008228805750159*mpos[1]-PI;
							jvel = 0*-0.001067944191703*mvel[0]+0.008228805750159*mvel[1];
						break;
						case 2:
						    jpos=0*-0.000048622484703*mpos[0]-0*0.000066464064044*mpos[1]
								  +0.000463265306122*mpos[2];
						    jvel=0*-0.000048622484703*mvel[0]-0*0.000066464064044*mvel[1]
							      +0.000463265306122*mvel[2];
						break;
					}
					r_state[i] = jpos;
					r_state[3+i] = jvel;
					r_state[6+i] = mpos[i];
					r_state[9+i] = mvel[i];
		  			/*printf("r_state[%d] = %f, %f, %f, %f\n",i,
						    r_state[i],r_state[3+i],r_state[6+i],r_state[9+i]);*/
				}
			}

			t_start = clock();
#ifdef EULER_INT
				sys_dyn_gold_euler(r_state, EULER_STEP*1000);
				//int detect = sys_dyn_gold_euler_detect(r_state, EULER_STEP*1000,iter_time_gold);
				//if (detect!=0)
				//{
					//cout<<detect<<endl;
				//}
#else
				integrate_adaptive(rk4(), sys_dyn_gold, r_state, 0.0, 0.001, RK_STEP);
#endif
			t_end = clock();

			iter_time_gold=iter_time_gold+1;
		}
		else
		{
			iter_time_green=iter_time_green+1;
		}

		double duration=(t_end-t_start)/1000.0;
		/*if (duration < 0)
		   cout << "Negative Duration!!! t1 = "<< (double)t_start/1000.0
				<< ", t2 = "<<(double)t_end/1000.0<<"\n";*/
		//cout << "Gold Arm Dynamics Calculation:" << duration << " ms\n";
		sum_d = sum_d + duration;
		//cout << "Sum:" << sum_d/1000.0 << " s\n";

	    //if (arm_type == 1)
	       	 //printf("------> GREEN ARM MODELS NOT AVAILABLE\n");
		//else
	     sprintf(buf, "%f %f %f %f %f %f %f %f %f",
				   r_state[6],r_state[9],r_state[0],r_state[7],r_state[10],r_state[1],r_state[8],r_state[11],r_state[2]);
	    //printf("%f %f %f %f %f %f", r_state[6],r_state[9],r_state[7],r_state[10],r_state[8],r_state[11]);
		//printf("Sent estimated motor positions/velocities and joint positions for %s arm:\n%s\n", 		(arm_type==1)?"Green":"Gold", buf);
        write(fd2, buf, sizeof(buf));
	}
    //write_sys(x);
	//cout << "Average Gold Arm Dynamics Calculation:" << sum_d/iter_time_green << " ms\n";

#ifdef LOGGING
    FILE *f = fopen("results", "a");
#ifdef EULER_INT
	fprintf(f,"Average Calculation Time for Euler %f = %f\n", EULER_STEP, sum_d/iter_time_gold);
#else
	fprintf(f,"Average Calculation Time for RK4 %f = %f\n", RK_STEP, sum_d/iter_time_gold);
#endif
#endif

	/* remove the FIFO */
    unlink(wrfifo);
    close(fd1);
    close(fd2);
	return 0;
}
