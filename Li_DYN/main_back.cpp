#include "two_arm_dyn.h"
#include <time.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>

#define MAX_BUF 4096

int iter_time_gold, iter_time_green=0;
state_type r_d_gold, r_d_green;
double V[4], Load[4];
int motor_id;
double scale[4] = {2.9,2.9,2.9,2.9};
double disp[4] = {48,48,48,48};
double init_mpos[4], init_est_mpos[4];

int main()
{
	struct timespec t1, t2;
	clock_t t_start,t_end;   
    double mpos[4],mvel[4],dac[4];
    int arm_type, packet_num;
	double sum_d = 0.0;
	int fd1,fd2;
    char rdfifo[20] = "/tmp/dac_fifo";
    char wrfifo[20] = "/tmp/mpos_vel_fifo";
    mkfifo(wrfifo, 0667);
    printf("Write FIFO Created..\n");    
	char buf[MAX_BUF];
    /* open, read, and display the message from the FIFO */
    fd1 = open(rdfifo, O_RDONLY);
    printf("Read FIFO Opened..\n");
    fd2 = open(wrfifo, O_WRONLY);
    printf("Write FIFO Opened..\n");
    motor_state m_state[4];
	for (int i = 0; i < 4; i++)
		m_state[i] = {0.0,0.0,0.0}; 

	while(read(fd1, buf, MAX_BUF))
	{       
        stringstream ss(buf);    
        ss >> arm_type >> packet_num >> 
			  mpos[0]>>mpos[1]>>mpos[2]>>mvel[0]>>mvel[1]>>mvel[2]>>dac[0]>>dac[1]>>dac[2];
		cout << "\n########## Packet #"<<packet_num<<" ##########\n";
        printf("Received motor velocities and dacs for %s arm:\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", (arm_type==1)?"Green":"Gold",mpos[0],mpos[1],mpos[2],mvel[0],mvel[1],mvel[2],dac[0],dac[1],dac[2]);
	    
	    if (arm_type == 0)
		{
			for (int i = 0; i < 3; i++)
			{			
				V[i] = dac[i]*0.0003051804379;
				//Load[i] = -1.9*1e-2*m_state[i][2]/scale[i];
				Load[i] = -1.9*1e-2*mvel[i];				
				printf("Load[%d] = %f\n",i,Load[i]);		    
			}			
			// initial mpos and state
		    if (iter_time_gold == 0)
			{
				for (int i = 0; i < 3; i++)	
				{
					init_mpos[i] = mpos[i];			
					m_state[i] = {0.4, mpos[i], mvel[i]};
					Load[i] = -1.9*1e-2*mvel[i];
		  			printf("m_state[%d] = %f, %f, %f\n",i,m_state[i][0],m_state[i][1],m_state[i][2]);
				}			
			}		    
			t_start = clock();
            motor_id = 0;
			integrate_adaptive(rk5_m(), motor_40, m_state[0], 0.0, 0.001, 0.0002); 
			motor_id = 1;
			integrate_adaptive(rk5_m(), motor_40, m_state[1], 0.0, 0.001, 0.0002); 				
     	    motor_id = 2;
			integrate_adaptive(rk5_m(), motor_40, m_state[2], 0.0, 0.001, 0.0002); 				
            //motor_id = 3;
			//integrate_adaptive(rk5_m(), motor_30, m_state[3], 0.0, 0.001, 0.0002); 
			t_end = clock();

			// initial est_mpos
		    if (iter_time_gold == 0)
			{
				for (int i = 0; i < 3; i++)	
					init_est_mpos[i] = m_state[i][1]/scale[i]+disp[i];
			}	 
			iter_time_gold=iter_time_gold+1;
		}
		else		
		{		
			iter_time_green=iter_time_green+1;	
		}
 
		double duration=(t_end-t_start)/1000.0;
		if (duration < 0)
		   cout << "Negative Duration!!! t1 = "<< (double)t_start/1000.0 
				<< ", t2 = "<<(double)t_end/1000.0<<"\n";
		cout << "Gold Arm Dynamics Calculation:" << duration << " ms\n";
		sum_d = sum_d + duration; 
		cout << "Sum:" << sum_d/1000.0 << " s\n";
		if (sum_d < 0)
		{
		   cout << "Negative Sum !!!!!!!!\n";
		   return 0;
		}

	    if (arm_type == 1)
	       	printf("------> GREEN ARM MODELS NOT AVAILABLE\n");
		else
	        sprintf(buf, "%f %f %f %f %f %f %f %f",
				m_state[0][1]/scale[0]+disp[0]+init_mpos[0]-init_est_mpos[0],m_state[0][2]/scale[0],
			    m_state[1][1]/scale[1]+disp[1]+init_mpos[1]-init_est_mpos[1],m_state[1][2]/scale[1],
   				m_state[2][1]/scale[2]+disp[2]+init_mpos[2]-init_est_mpos[2],m_state[2][2]/scale[2]);
       
		printf("Sent estimated motor positions/velocities for %s arm:\n%s\n", 		(arm_type==1)?"Green":"Gold", buf);
        write(fd2, buf, sizeof(buf));	
	}
    //write_sys(x);
	cout << "Average Gold Arm Dynamics Calculation:" << sum_d/iter_time_green << " ms\n";

    /* remove the FIFO */
    unlink(wrfifo);
    close(fd1);
    close(fd2);
	return 0;
}
