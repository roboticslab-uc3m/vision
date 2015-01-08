// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HEAD_BOT__
#define __HEAD_BOT__

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <stdlib.h>  // just for exit()

#include "ColorDebug.hpp"

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

/** RTAI PROCESS BOILERPLATE **/
static void* rt_system_thread(void * arg);
static double targetDegs[NDOF_HEAD];

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

class HeadBot : public DeviceDriver, public IPositionControl, public IVelocityControl, public IEncoders {

  public:

  // ------- IPositionControl declarations. -------

    /**
     * Get the number of controlled axes. This command asks the number of controlled
     * axes for the current physical interface.
     * @param ax pointer to storage
     * @return true/false.
     */
    virtual bool getAxes(int *ax);

    /** Set position mode. This command
     * is required by control boards implementing different
     * control methods (e.g. velocity/torque), in some cases
     * it can be left head.
     * return true/false on success/failure
     */
    virtual bool setPositionMode();

    /** Set new reference point for a single axis.
     * @param j joint number
     * @param ref specifies the new ref point
     * @return true/false on success/failure
     */
    virtual bool positionMove(int j, double ref);

    /** Set new reference point for all axes.
     * @param refs array, new reference points.
     * @return true/false on success/failure
     */
    virtual bool positionMove(const double *refs);

    /** Set relative position. The command is relative to the 
     * current position of the axis.
     * @param j joint axis number
     * @param delta relative command
     * @return true/false on success/failure
     */
    virtual bool relativeMove(int j, double delta);

    /** Set relative position, all joints.
     * @param deltas pointer to the relative commands
     * @return true/false on success/failure
     */
    virtual bool relativeMove(const double *deltas);

    /** Check if the current trajectory is terminated. Non blocking.
     * @return true if the trajectory is terminated, false otherwise
     */
    virtual bool checkMotionDone(int j, bool *flag);

    /** Check if the current trajectory is terminated. Non blocking.
     * @return true if the trajectory is terminated, false otherwise
     */
    virtual bool checkMotionDone(bool *flag);

    /** Set reference speed for a joint, this is the speed used during the
     * interpolation of the trajectory.
     * @param j joint number
     * @param sp speed value
     * @return true/false upon success/failure
     */
    virtual bool setRefSpeed(int j, double sp);

    /** Set reference speed on all joints. These values are used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array of speed values.
     * @return true/false upon success/failure
     */
    virtual bool setRefSpeeds(const double *spds);

    /** Set reference acceleration for a joint. This value is used during the
     * trajectory generation.
     * @param j joint number
     * @param acc acceleration value
     * @return true/false upon success/failure
     */
    virtual bool setRefAcceleration(int j, double acc);

    /** Set reference acceleration on all joints. This is the valure that is
     * used during the generation of the trajectory.
     * @param accs pointer to the array of acceleration values
     * @return true/false upon success/failure
     */
    virtual bool setRefAccelerations(const double *accs);

    /** Get reference speed for a joint. Returns the speed used to 
     * generate the trajectory profile.
     * @param j joint number
     * @param ref pointer to storage for the return value
     * @return true/false on success or failure
     */
    virtual bool getRefSpeed(int j, double *ref);

    /** Get reference speed of all joints. These are the  values used during the
     * interpolation of the trajectory.
     * @param spds pointer to the array that will store the speed values.
     */
    virtual bool getRefSpeeds(double *spds);

    /** Get reference acceleration for a joint. Returns the acceleration used to 
     * generate the trajectory profile.
     * @param j joint number
     * @param acc pointer to storage for the return value
     * @return true/false on success/failure
     */
    virtual bool getRefAcceleration(int j, double *acc);

    /** Get reference acceleration of all joints. These are the values used during the
     * interpolation of the trajectory.
     * @param accs pointer to the array that will store the acceleration values.
     * @return true/false on success or failure 
     */
    virtual bool getRefAccelerations(double *accs);

    /** Stop motion, single joint
     * @param j joint number
     * @return true/false on success/failure
     */
    virtual bool stop(int j);

    /** Stop motion, multiple joints 
     * @return true/false on success/failure
     */
    virtual bool stop();

    //  ---------- IEncoder Declarations. ----------

    /**
     * Reset encoder, single joint. Set the encoder value to zero 
     * @param j encoder number
     * @return true/false
     */
    virtual bool resetEncoder(int j);

    /**
     * Reset encoders. Set the encoders value to zero 
     * @return true/false
     */
    virtual bool resetEncoders();

    /**
     * Set the value of the encoder for a given joint. 
     * @param j encoder number
     * @param val new value
     * @return true/false
     */
    virtual bool setEncoder(int j, double val);

    /**
     * Set the value of all encoders.
     * @param vals pointer to the new values
     * @return true/false
     */
    virtual bool setEncoders(const double *vals);

    /**
     * Read the value of an encoder.
     * @param j encoder number
     * @param v pointer to storage for the return value
     * @return true/false, upon success/failure (you knew it, uh?)
     */
    virtual bool getEncoder(int j, double *v);

    /**
     * Read the position of all axes.
     * @param encs pointer to the array that will contain the output
     * @return true/false on success/failure
     */
    virtual bool getEncoders(double *encs);

    /**
     * Read the istantaneous speed of an axis.
     * @param j axis number
     * @param sp pointer to storage for the output
     * @return true if successful, false ... otherwise.
     */
    virtual bool getEncoderSpeed(int j, double *sp);

    /**
     * Read the instantaneous speed of all axes.
     * @param spds pointer to storage for the output values
     * @return guess what? (true/false on success or failure).
     */
    virtual bool getEncoderSpeeds(double *spds);
    
    /**
     * Read the instantaneous acceleration of an axis.
     * @param j axis number
     * @param spds pointer to the array that will contain the output
     */
    virtual bool getEncoderAcceleration(int j, double *spds);

    /**
     * Read the instantaneous acceleration of all axes.
     * @param accs pointer to the array that will contain the output
     * @return true if all goes well, false if anything bad happens. 
     */
    virtual bool getEncoderAccelerations(double *accs);

    //  --------- IVelocityControl Declarations.  ---------

    /**
     * Set velocity mode. This command
     * is required by control boards implementing different
     * control methods (e.g. velocity/torque), in some cases
     * it can be left head.
     * @return true/false on success failure
     */
    virtual bool setVelocityMode();

    /**
     * Start motion at a given speed, single joint.
     * @param j joint number
     * @param sp speed value
     * @return bool/false upone success/failure
     */
    virtual bool velocityMove(int j, double sp);

    /**
     * Start motion at a given speed, multiple joints.
     * @param sp pointer to the array containing the new speed values
     * @return true/false upon success/failure
     */
    virtual bool velocityMove(const double *sp);

    // -------- DeviceDriver declarations.  --------

    /**
     * Open the DeviceDriver. 
     * @param config is a list of parameters for the device.
     * Which parameters are effective for your device can vary.
     * See \ref dev_examples "device invocation examples".
     * If there is no example for your device,
     * you can run the "yarpdev" program with the verbose flag
     * set to probe what parameters the device is checking.
     * If that fails too,
     * you'll need to read the source code (please nag one of the 
     * yarp developers to add documentation for your device).
     * @return true/false upon success/failure
     */
    virtual bool open(Searchable& config);

    /**
     * Close the DeviceDriver.
     * @return true/false on success/failure.
     */
    virtual bool close();

  // ------------------------------- Private -------------------------------------

  private:

        /** Check if index is within range (referred to driver vector size).
         * @param idx index to check.
         * @return true/false on success/failure.
         */
        bool indexWithinRange(const int& idx);

        int numMotors;

};

}  // namespace teo

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
        cmd.head.q_desired[0] = targetDegs[0];  //-- Desired position (rad).
        cmd.head.slew_rate_q_desired[0] = 0.5;  //-- Desired velocity (rad/s).
        cmd.head.q_stiffness[0] = 0.5;  //0-1.0
        cmd.head.smoothing_mode[0] = SMOOTHING_MODE_SLEW;  //-- Smooth trajectory.

        cmd.head.ctrl_mode[1] = JOINT_ARRAY_MODE_THETA;
        cmd.head.q_desired[1] = targetDegs[1];  //-- Desired position (rad).
        cmd.head.slew_rate_q_desired[1] = 1.0;  //-- Desired velocity (rad/s).
        cmd.head.q_stiffness[1] = 1.0;  //0-1.0
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


#endif  // __HEAD_BOT__
