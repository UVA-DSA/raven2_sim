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


/**
* \file log.cpp
* \brief Generic logging function
*
*/

#include <stdio.h>
#include <stdarg.h>
#include <ros/console.h>
#include <defines.h>
#include <queue>

std::queue<char*> msgqueue;
const static size_t MAX_MSG_LEN =1024;

//#define simulator_logging
#ifdef save_logs
#include <fstream>
extern char* raven_path;
extern int inject_mode;
extern int logging;
extern char err_str[1024];
/**\fn int log_file(const char* fmt,...)
*  \brief
*  \param fmt
*  \return 0 on success -1 on failure
*/

int log_file(const char* fmt,...)
{
		static char buf[MAX_MSG_LEN];
		va_list args;
		va_start (args, fmt);
		//Do something
		vsprintf(buf,fmt,args);
		va_end(args);
#ifndef no_logging
  if (logging == 1)
	{
 		std::ofstream logfile;
    static char buff[50];
		if (inject_mode == 0)
        sprintf(buff,"%s/sim_log.txt", raven_path);
    else
		    sprintf(buff,"%s/fault_log_%d.txt", raven_path, inject_mode);
    logfile.open(buff,std::ofstream::out | std::ofstream::app);
		// Log in the file
		logfile << buf << "\n";
		logfile.close();
  }
#endif
	if ((strstr(buf,"ERROR") || strstr(buf,"Error") || strstr(buf,"rror")) && !strstr(buf,"Packet"))
	{
	   sprintf(buf, "%s;",buf);
       if (err_str[0] != '\0')
	       strcat(err_str, buf);
	   else
		   strcpy(err_str, buf);
	   //printf("%s\n",err_str);
	}
	// Print on console
	//ROS_INFO("%s",buf);
  return 0;
}
#endif

/**\fn int log_msg_later(const char* fmt,...)
*  \brief
*  \param fmt
*  \return 0 on success -1 on failure
*/

int log_msg_later(const char* fmt,...)
{
#ifndef no_logging
  static char buf[MAX_MSG_LEN];
  char* msgbuf;
  va_list args;
  va_start (args, fmt);

  //Do somethin
  vsprintf(buf,fmt,args);
  va_end(args);
//    printf("%s",buf);
  msgbuf = (char*)malloc(strlen(buf)+1);
  strcpy(msgbuf, buf);
  msgqueue.push(msgbuf);
  //  ROS_INFO("%s",buf);
  //std::cout << "qsiz:" << msgqueue.size() << std::endl;
#endif
  return 0;
}

/**\fn int log_msg(const char* fmt,...)
*  \brief
*  \param fmt
*  \return 0 on success -1 on failure
*/

int log_msg(const char* fmt,...)
{
#ifndef no_logging
  static char buf[MAX_MSG_LEN];
    va_list args;
    va_start (args, fmt);
    //Do somethinh
    vsprintf(buf,fmt,args);
    va_end(args);
//    printf("%s",buf);
    ROS_INFO("%s",buf);
#endif
    return 0;
}

/**\fn int err_msg(const char* fmt,...)
*  \brief
*  \param fmt
*  \return 0 on success -1 on failure
*/

int err_msg(const char* fmt,...)
{
#ifndef no_logging
  static char buf[MAX_MSG_LEN];
    va_list args;
    va_start (args, fmt);
    //Do somethinh
    vsprintf(buf,fmt,args);
    va_end(args);
//    printf("%s",buf);
    ROS_ERROR("%s",buf);
#endif
    return 0;
}
