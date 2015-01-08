// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_HEAD_HPP__
#define __TEO_HEAD_HPP__

/* 
M3 -- Meka Robotics Robot Components
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/
 
#include <rtai_sched.h>
#include <stdio.h>
#include <signal.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
#include <m3rt/base/m3ec_def.h>
#include <m3rt/base/m3rt_def.h>
#include <rtai_nam2num.h>
#include <rtai_registry.h>
#include "m3/robots/humanoid_shm_sds.h"
#include "m3/hardware/joint_mode_ros.pb.h"
#include "m3/robots/chain_name.h"
#include <string>
#include <iomanip>
#include <locale>
#include <sstream>
#include <iostream>
#include <fstream>
//#include <m3rt/base/toolbox.h>

#define NDOF_HEAD 2

#define RT_TASK_FREQUENCY_MEKA_OMNI_SHM 100
#define RT_TIMER_TICKS_NS_MEKA_OMNI_SHM (1000000000 / RT_TASK_FREQUENCY_MEKA_OMNI_SHM)		//Period of rt-timer 
#define MEKA_ODOM_SHM "TSHMM"
#define MEKA_ODOM_CMD_SEM "TSHMC"
#define MEKA_ODOM_STATUS_SEM "TSHMS"

static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static M3HumanoidShmSdsCommand cmd;
static M3HumanoidShmSdsStatus status;
static int sds_status_size;
static int sds_cmd_size;
static long step_cnt = 0;
static void endme(int dummy) { std::cout << "END\n"; end=1; }
//m3ctrl_msgs::M3JointCmd joint_cmd_g;


/** RTAI PROCESS BOILERPLATE **/
static void* rt_system_thread(void * arg)
{
    SEM * status_sem;
    SEM * command_sem;
    RT_TASK *task;
    int cntr=0;
    M3Sds * sds = (M3Sds *)arg;
    printf("Starting real-time thread\n");

    sds_status_size = sizeof(status);
    sds_cmd_size = sizeof(cmd);

    memset(&cmd, 0, sds_cmd_size);

    task = rt_task_init_schmod(nam2num("HSHMP"), 0, 0, 0, SCHED_FIFO, 0xF);
    rt_allow_nonroot_hrt();
    if (task==NULL)
    {
        printf("Failed to create RT-TASK TSHMP\n");
        return 0;
    }
    status_sem=(SEM*)rt_get_adr(nam2num(MEKA_ODOM_STATUS_SEM));
    command_sem=(SEM*)rt_get_adr(nam2num(MEKA_ODOM_CMD_SEM));
    if (!status_sem)
    {
        printf("Unable to find the %s semaphore.\n",MEKA_ODOM_STATUS_SEM);
        rt_task_delete(task);
        return 0;
    }
    if (!command_sem)
    {
        printf("Unable to find the %s semaphore.\n",MEKA_ODOM_CMD_SEM);
        rt_task_delete(task);
        return 0;
    }

    RTIME tick_period = nano2count(RT_TIMER_TICKS_NS_MEKA_OMNI_SHM);
    RTIME now = rt_get_time();
    rt_task_make_periodic(task, now + tick_period, tick_period);
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_hard_real_time();
    long long start_time, end_time, dt;
    long long step_cnt = 0;
    sys_thread_active=1;

    while(!sys_thread_end)
    {
        start_time = nano2count(rt_get_cpu_time_ns());

        //-- Read encoder update
        rt_sem_wait(status_sem);
        memcpy(&status, sds->status, sds_status_size);
        rt_sem_signal(status_sem);
        //-- Read encoder do (better do it in main)
        //printf("0: %f | 1: %f | Time: %lld\n", status.head.theta[0], status.head.theta[1], status.timestamp);

        //-- Move motor command (must be done here):
        cmd.timestamp = status.timestamp;

        cmd.head.ctrl_mode[0] = JOINT_ARRAY_MODE_THETA;
        cmd.head.q_desired[0] = 0.0;  //-- Desired position (rad).
        cmd.head.slew_rate_q_desired[0] = 0.5;  //-- Desired velocity (rad/s).
        cmd.head.q_stiffness[0] = 0.5;  //0-1.0
        cmd.head.smoothing_mode[0] = SMOOTHING_MODE_SLEW;  //-- Smooth trajectory.

        cmd.head.ctrl_mode[1] = JOINT_ARRAY_MODE_THETA;
        cmd.head.q_desired[1] = 0.0;  //-- Desired position (rad).
        cmd.head.slew_rate_q_desired[1] = 0.5;  //-- Desired velocity (rad/s).
        cmd.head.q_stiffness[1] = 0.5;  //0-1.0
        cmd.head.smoothing_mode[1] = SMOOTHING_MODE_SLEW;  //-- Smooth trajectory.

        //-- Move motor do
        rt_sem_wait(command_sem);
        memcpy(sds->cmd, &cmd, sds_cmd_size);
        rt_sem_signal(command_sem);

        end_time = nano2count(rt_get_cpu_time_ns());
        dt=end_time-start_time;
        /*
        Check the time it takes to run components, and if it takes longer
        than our period, make us run slower. Otherwise this task locks
        up the CPU.*/
        if (dt > tick_period && step_cnt>10)
        {
            printf("Step %lld: Computation time of components is too long. Forcing all components to state SafeOp.\n",step_cnt);
            printf("Previous period: %f. New period: %f\n", (double)count2nano(tick_period),(double)count2nano(dt));
            tick_period=dt;
            //rt_task_make_periodic(task, end + tick_period,tick_period);
        }
        step_cnt++;

        rt_task_wait_period();
    }
    printf("Exiting RealTime Thread...\n",0);
    rt_make_soft_real_time();
    rt_task_delete(task);
    sys_thread_active=0;
    return 0;
}

#endif  // __TEO_HEAD_HPP__
