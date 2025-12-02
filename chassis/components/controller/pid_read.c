#include "pid_read.h"
#include "chassis_task.h"

extern Chassis_t chassis;

Read_PID_Data_t read_pid;

void Read_PID(fp32 data)
{
	if(read_pid.read_flag == OPEN_)
	{
			if((read_pid.count/50)>=0 && (read_pid.count/50)<30)
			{
				if(read_pid.count%50 == 0)
					read_pid.pid_data[read_pid.count/50] = data;
				read_pid.count ++;
			}
			else read_pid.read_flag = SHUT_;
	}
	else if(read_pid.read_flag == READY_)
	{
		read_pid.count = 0;
		if(chassis.no_force_mode == SHUT)
			read_pid.read_flag = OPEN_;
	}
}			

