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
#include "yaml-cpp/yaml.h"
//#include <m3rt/base/toolbox.h>

// Needed for ROS
#include <ros/ros.h>
#include <m3ctrl_msgs/M3JointCmd.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


#define RT_TASK_FREQUENCY_MEKA_OMNI_SHM 100
#define RT_TIMER_TICKS_NS_MEKA_OMNI_SHM (1000000000 / RT_TASK_FREQUENCY_MEKA_OMNI_SHM)		//Period of rt-timer 
#define MEKA_ODOM_SHM "TSHMM"
#define MEKA_ODOM_CMD_SEM "TSHMC"
#define MEKA_ODOM_STATUS_SEM "TSHMS"

static int ndof_head = 0;
static int ndof_right_arm = 0;
static int ndof_left_arm = 0;
static int ndof_torso = 0;
static int ndof_right_hand = 0;
static int ndof_left_hand = 0;
static int ndof_right_gripper = 0;
static int ndof_left_gripper = 0;

static int ndof_total = 0;


using namespace std;

////////////////////////////////////////////////////////////////////////////////////
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
sensor_msgs::JointState joint_state_g;
m3ctrl_msgs::M3JointCmd joint_cmd_g;
ros::Publisher publisher_g;

ros::Subscriber cmd_sub_g;

ros::Subscriber payload_sub_g;
//tf::TransformBroadcaster odom_broadcaster;
////////////////////////////////////////////////////////////////////////////////////


///////  Periodic Control Loop:
void StepShm();
void commandCallback(const m3ctrl_msgs::M3JointCmdConstPtr& msg);

///////////////////////////////

void SetTimestamp(int64_t  timestamp)
{
  cmd.timestamp = timestamp;
    return; 
}

int64_t GetTimestamp()
{  
    return status.timestamp; 
}

////////////////////////// MAIN COMPUTATION METHOD /////////////////////////////

void StepShm(int cntr)
{   
  
    SetTimestamp(GetTimestamp()); //Pass back timestamp as a heartbeat

    joint_state_g.header.stamp = ros::Time::now();  
    
     int j = 0;
  for (int i = 0; i < ndof_right_arm; i++)
  {
    joint_state_g.position[j] = DEG2RAD(status.right_arm.theta[i]);
    joint_state_g.velocity[j] = DEG2RAD(status.right_arm.thetadot[i]);
    joint_state_g.effort[j] = status.right_arm.torque[i];
    j++;
    
  }

  for (int i = 0; i < ndof_left_arm; i++)
  {
    joint_state_g.position[j] = DEG2RAD(status.left_arm.theta[i]);
    joint_state_g.velocity[j] = DEG2RAD(status.left_arm.thetadot[i]);
    joint_state_g.effort[j] = status.left_arm.torque[i];
    j++;  
  }

  for (int i = 0; i < ndof_head; i++)
  {
    joint_state_g.position[j] = DEG2RAD(status.head.theta[i]);
    joint_state_g.velocity[j] = DEG2RAD(status.head.thetadot[i]);
    joint_state_g.effort[j] = status.head.torque[i];
      j++;
  }

  for (int i = 0; i < ndof_torso; i++)
  {
    joint_state_g.position[j] = DEG2RAD(status.torso.theta[i]);
    joint_state_g.velocity[j] = DEG2RAD(status.torso.thetadot[i]);
    joint_state_g.effort[j] = status.torso.torque[i];
      j++;
  }
  
  for (int i = 0; i < ndof_right_hand; i++)
  {
    joint_state_g.position[j] = DEG2RAD(status.right_hand.theta[i]);
    joint_state_g.velocity[j] = DEG2RAD(status.right_hand.thetadot[i]);
    joint_state_g.effort[j] = status.right_hand.torque[i];
      j++;
  }
  
  for (int i = 0; i < ndof_left_hand; i++)
  {
    joint_state_g.position[j] = DEG2RAD(status.left_hand.theta[i]);
    joint_state_g.velocity[j] = DEG2RAD(status.left_hand.thetadot[i]);
    joint_state_g.effort[j] = status.left_hand.torque[i];
      j++;
  }
  
  for (int i = 0; i < ndof_left_gripper; i++)
  {
    joint_state_g.position[j] = DEG2RAD(status.left_gripper.theta[i]);
    joint_state_g.velocity[j] = DEG2RAD(status.left_gripper.thetadot[i]);
    joint_state_g.effort[j] = status.left_gripper.torque[i];
      j++;
  }
  
  
    publisher_g.publish(joint_state_g);
    
     /*if (cntr % 100 == 0)
      {	
	if (1)
	{
	  printf("********************************\n");
	  printf("timestamp: %ld\n", status.timestamp);	  
	  {	    	    
	    printf("------------------------------\n");
	    printf("position: %f\n", status.position);
	    printf("velocity: %f\n", status.velocity);
	    printf("effort: %f\n", status.effort);	    
	     printf("------------------------------\n");
	    printf("\n");
	  }
	}
      }*/
    
     /*cmd.right_arm.q_desired[0] = 0;
      cmd.right_arm.slew_rate_q_desired[0] = 30;      
      cmd.right_arm.q_stiffness[0] = 1.0; 
      cmd.right_arm.ctrl_mode[0] = JOINT_ARRAY_MODE_THETA_GC; 
      cmd.right_arm.smoothing_mode[0] = SMOOTHING_MODE_SLEW; */
 
   
}

void commandCallback(const m3ctrl_msgs::M3JointCmdConstPtr& msg)
{
    
  
  for (int i = 0; i < msg->chain.size(); i++)
  {
    int chain_idx = msg->chain_idx[i];
    if ((M3Chain)msg->chain[i] == RIGHT_ARM)
    {
      if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_OFF)
	cmd.right_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA)
	cmd.right_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA_GC)
	cmd.right_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA_GC; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_POSE)
	cmd.right_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_POSE; 
      else
	cmd.right_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
      
      cmd.right_arm.q_desired[chain_idx] = msg->position[i];
      cmd.right_arm.slew_rate_q_desired[chain_idx] = msg->velocity[i];      
      cmd.right_arm.q_stiffness[chain_idx] = msg->stiffness[i];       
      cmd.right_arm.smoothing_mode[chain_idx] = (SMOOTHING_MODE)msg->smoothing_mode[i]; 
    } 
    else if ((M3Chain)msg->chain[i] == LEFT_ARM)
    {
      if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_OFF)
	cmd.left_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA)
	cmd.left_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA_GC)
	cmd.left_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA_GC; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_POSE)
	cmd.left_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_POSE;       
      else
	cmd.left_arm.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 

      cmd.left_arm.q_desired[chain_idx] = msg->position[i];
      cmd.left_arm.slew_rate_q_desired[chain_idx] = msg->velocity[i];      
      cmd.left_arm.q_stiffness[chain_idx] = msg->stiffness[i];       
      cmd.left_arm.smoothing_mode[chain_idx] = (SMOOTHING_MODE)msg->smoothing_mode[i]; 
    } 
    else if ((M3Chain)msg->chain[i] == HEAD)
    {
      if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_OFF)
	cmd.head.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA)
	cmd.head.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA;       
      else
	cmd.head.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
      cmd.head.q_desired[chain_idx] = msg->position[i];
      cmd.head.slew_rate_q_desired[chain_idx] = msg->velocity[i];      
      cmd.head.q_stiffness[chain_idx] = msg->stiffness[i];       
      cmd.head.smoothing_mode[chain_idx] = (SMOOTHING_MODE)msg->smoothing_mode[i]; 
      //printf("here!\n");
    }
    else if ((M3Chain)msg->chain[i] == TORSO)
    {
       if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_OFF)
	cmd.torso.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA)
	cmd.torso.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA_GC)
	cmd.torso.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA_GC; 
      else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_POSE)
	cmd.torso.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_POSE;       
      else
	cmd.torso.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
      cmd.torso.q_desired[chain_idx] = msg->position[i];
      cmd.torso.slew_rate_q_desired[chain_idx] = msg->velocity[i];      
      cmd.torso.q_stiffness[chain_idx] = msg->stiffness[i]; 
      cmd.torso.smoothing_mode[chain_idx] = (SMOOTHING_MODE)msg->smoothing_mode[i]; 
    }      
    else if ((M3Chain)msg->chain[i] == RIGHT_HAND)
    {
	if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_OFF)
	  cmd.right_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA)
	  cmd.right_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA_GC)
	  cmd.right_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA_GC; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_TORQUE_GC)
	  cmd.right_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_TORQUE_GC; 
	else
	  cmd.right_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
	cmd.right_hand.q_desired[chain_idx] = msg->position[i];
	cmd.right_hand.slew_rate_q_desired[chain_idx] = msg->velocity[i];      
	cmd.right_hand.q_stiffness[chain_idx] = msg->stiffness[i]; 
	cmd.right_hand.smoothing_mode[chain_idx] = (SMOOTHING_MODE)msg->smoothing_mode[i]; 	
	cmd.right_hand.tq_desired[chain_idx] = msg->effort[i]; 
//	printf("mo: %d \n",  (int)cmd.right_hand.ctrl_mode[chain_idx]);
    }      
    else if ((M3Chain)msg->chain[i] == LEFT_HAND)
    {
	if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_OFF)
	  cmd.left_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA)
	  cmd.left_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA_GC)
	  cmd.left_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA_GC; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_TORQUE_GC)
	  cmd.left_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_TORQUE_GC; 
	else
	  cmd.left_hand.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
	cmd.left_hand.q_desired[chain_idx] = msg->position[i];
	cmd.left_hand.slew_rate_q_desired[chain_idx] = msg->velocity[i];      
	cmd.left_hand.q_stiffness[chain_idx] = msg->stiffness[i]; 	
	cmd.left_hand.smoothing_mode[chain_idx] = (SMOOTHING_MODE)msg->smoothing_mode[i]; 
	cmd.left_hand.tq_desired[chain_idx] = msg->effort[i]; 
     }
     else if ((M3Chain)msg->chain[i] == LEFT_GRIPPER)
    {
	if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_OFF)
	  cmd.left_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA)
	  cmd.left_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA;
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA_GC)
	  cmd.left_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA_GC; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_TORQUE)
	  cmd.left_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_TORQUE; 
	else
	  cmd.left_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
	cmd.left_gripper.q_desired[chain_idx] = msg->position[i];
	cmd.left_gripper.slew_rate_q_desired[chain_idx] = msg->velocity[i];      
	cmd.left_gripper.q_stiffness[chain_idx] = msg->stiffness[i]; 	
	cmd.left_gripper.smoothing_mode[chain_idx] = (SMOOTHING_MODE)msg->smoothing_mode[i]; 
	cmd.left_gripper.tq_desired[chain_idx] = msg->effort[i]; 
     }
     else if ((M3Chain)msg->chain[i] == RIGHT_GRIPPER)
    {
	if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_OFF)
	  cmd.right_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA)
	  cmd.right_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA;
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_THETA_GC)
	  cmd.right_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_THETA_GC; 
	else if ((JOINT_MODE_ROS)msg->control_mode[i] == JOINT_MODE_ROS_TORQUE)
	  cmd.right_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_TORQUE; 
	else
	  cmd.right_gripper.ctrl_mode[chain_idx] = JOINT_ARRAY_MODE_OFF; 
	cmd.right_gripper.q_desired[chain_idx] = msg->position[i];
	cmd.right_gripper.slew_rate_q_desired[chain_idx] = msg->velocity[i];      
	cmd.right_gripper.q_stiffness[chain_idx] = msg->stiffness[i]; 	
	cmd.right_gripper.smoothing_mode[chain_idx] = (SMOOTHING_MODE)msg->smoothing_mode[i]; 
	cmd.right_gripper.tq_desired[chain_idx] = msg->effort[i]; 
     }
  }
  
      
}

sensor_msgs::JointState GetInitialJointStateMessage()
{
//Hard-coded for now
/*std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string[] name
float64[] position
float64[] velocity
float64[] effort
*/
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.header.frame_id = "humanoid";

  joint_state.name.resize(ndof_total);
  joint_state.position.resize(ndof_total,0.0);
  joint_state.velocity.resize(ndof_total,0.0);
  joint_state.effort.resize(ndof_total,0.0);
  
  
  

  int j = 0;
  for (int i = 0; i < ndof_right_arm; i++)
  {
    std::ostringstream convert;
    std::string int_str; 
    convert << i;
    int_str = convert.str();
    joint_state.name[j] = std::string("right_arm_j") + int_str;
    j++;
    
  }

  for (int i = 0; i < ndof_left_arm; i++)
  {
    std::ostringstream convert;
    std::string int_str; 
    convert << i;
    int_str = convert.str();
    joint_state.name[j] = std::string("left_arm_j") + int_str;
    j++;  
  }

  for (int i = 0; i < ndof_head; i++)
  {
    std::ostringstream convert;
    std::string int_str; 
    convert << i;
    int_str = convert.str();
    joint_state.name[j] = std::string("head_j") + int_str;
      j++;
  }

  for (int i = 0; i < ndof_torso; i++)
  {
    std::ostringstream convert;
    std::string int_str; 
    convert << i;
    int_str = convert.str();
    joint_state.name[j] = std::string("torso_j") + int_str;
      j++;
  }
  
  for (int i = 0; i < ndof_right_hand; i++)
  {
    std::ostringstream convert;
    std::string int_str; 
    convert << i;
    int_str = convert.str();
    joint_state.name[j] = std::string("right_hand_j") + int_str;
      j++;
  }
  
  for (int i = 0; i < ndof_left_hand; i++)
  {
    std::ostringstream convert;
    std::string int_str; 
    convert << i;
    int_str = convert.str();
    joint_state.name[j] = std::string("left_hand_j") + int_str;
      j++;
  }
  
   for (int i = 0; i < ndof_left_gripper; i++)
  {
    std::ostringstream convert;
    std::string int_str; 
    convert << i;
    int_str = convert.str();
    joint_state.name[j] = std::string("left_gripper_j") + int_str;
      j++;
  }


  return joint_state;
}

bool GetEnvironmentVar(const char * var, string &s)
{
	char *p=getenv(var);
	if (p!=NULL)
	{
		s.assign(p);
		return true;
	}
	return false;
}

void GetYamlDoc(const char * filename, YAML::Node & doc)
{	
	string s(filename);
	string path;
	
	if (GetEnvironmentVar("M3_ROBOT", path))
	{		
		path.append("/robot_config/");
		path.append(s);
	}
	ifstream fin(path.c_str());
	if (fin.fail())
	{		
		printf("could not read %s \n", path.c_str());	
	}

   	YAML::Parser parser(fin);
   	
   	parser.GetNextDocument(doc);
	fin.close();
	return;
}

////////////////////////// RTAI PROCESS BOILERPLATE /////////////////////////////

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
		rt_sem_wait(status_sem);
		memcpy(&status, sds->status, sds_status_size);		
		rt_sem_signal(status_sem);
		
		StepShm(cntr);		
		
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



////////////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{	
	RT_TASK *task;
	M3Sds * sys;
	int cntr=0;
	
	YAML::Node doc;
	GetYamlDoc("shm_humanoid_config.yml", doc);
	
	
	int ndof;
	
	try {
	  doc["ndof_chains"]["right_arm"] >> ndof;
	} catch (YAML::KeyNotFound &e) {
	  ndof = 0;	  
	}
	ndof_right_arm = ndof;
	ndof_total += ndof;
	
	try {
	  doc["ndof_chains"]["left_arm"] >> ndof;
	} catch (YAML::KeyNotFound &e) {
	  ndof = 0;	  
	}
	ndof_left_arm = ndof;
	ndof_total += ndof;
	
	try {
	  doc["ndof_chains"]["torso"] >> ndof;
	} catch (YAML::KeyNotFound &e) {
	  ndof = 0;	  
	}
	ndof_torso = ndof;
	ndof_total += ndof;
	
	try {
	  doc["ndof_chains"]["head"] >> ndof;
	} catch (YAML::KeyNotFound &e) {
	  ndof = 0;	  
	}
	ndof_head = ndof;
	ndof_total += ndof;
	
	try {
	  doc["ndof_chains"]["right_hand"] >> ndof;
	} catch (YAML::KeyNotFound &e) {
	  ndof = 0;	  
	}
	ndof_right_hand = ndof;
	ndof_total += ndof;
	
	try {
	  doc["ndof_chains"]["left_hand"] >> ndof;
	} catch (YAML::KeyNotFound &e) {
	  ndof = 0;	  
	}
	ndof_left_hand = ndof;
	ndof_total += ndof;
		
	try {
	  doc["ndof_chains"]["left_gripper"] >> ndof;
	} catch (YAML::KeyNotFound &e) {
	  ndof = 0;	  
	}
	ndof_left_gripper = ndof;
	ndof_total += ndof;

	
	//printf("ndof: %d\n", ndof_total);
	
	
	rt_allow_nonroot_hrt();
	
	/*ros::init(argc, argv, "base_controller"); // initialize ROS node
  	ros::AsyncSpinner spinner(1); // Use 1 thread - check if you actually need this for only publishing
  	spinner.start();
        ros::NodeHandle root_handle;*/
	
	ros::init(argc, argv, "hum_controller", ros::init_options::NoSigintHandler); // initialize ROS node
	ros::AsyncSpinner spinner(1); // Use 1 thread - check if you actually need this for only publishing
	spinner.start();
        ros::NodeHandle root_handle;
	ros::NodeHandle p_nh("~");
	
	cmd_sub_g = root_handle.subscribe<m3ctrl_msgs::M3JointCmd>("humanoid_command", 1, &commandCallback);
	
	joint_state_g = GetInitialJointStateMessage();
	publisher_g = root_handle.advertise<sensor_msgs::JointState>("humanoid_state", 1, true);

	signal(SIGINT, endme);

	if (sys = (M3Sds*)rt_shm_alloc(nam2num(MEKA_ODOM_SHM),sizeof(M3Sds),USE_VMALLOC))
		printf("Found shared memory starting shm_humanoid_controller.");
	else
	{
		printf("Rtai_malloc failure for %s\n",MEKA_ODOM_SHM);
		return 0;
	}

	rt_allow_nonroot_hrt();
	/*if (!(task = rt_task_init_schmod(nam2num("TSHM"), RT_TASK_PRIORITY, 0, 0, SCHED_FIFO, 0xF)))
	{
		rt_shm_free(nam2num(TORQUE_SHM));
		printf("Cannot init the RTAI task %s\n","TSHM");
		return 0;
	}*/
	hst=rt_thread_create((void*)rt_system_thread, sys, 10000);
	usleep(100000); //Let start up
	if (!sys_thread_active)
	{
		rt_task_delete(task);
		rt_shm_free(nam2num(MEKA_ODOM_SHM));
		printf("Startup of thread failed.\n",0);
		return 0;
	}
	while(!end)
	{		
		usleep(250000);
		
	}
	printf("Removing RT thread...\n",0);
	sys_thread_end=1;
	//rt_thread_join(hst);
	usleep(1250000);	
	if (sys_thread_active)printf("Real-time thread did not shutdown correctly\n");
	//rt_task_delete(task);
	rt_shm_free(nam2num(MEKA_ODOM_SHM));
	ros::shutdown();	
	return 0;
}

