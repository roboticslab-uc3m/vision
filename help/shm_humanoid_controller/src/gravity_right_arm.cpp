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

#include <iostream>

#include <ros/ros.h>

#include <m3ctrl_msgs/M3JointCmd.h>
#include "m3/hardware/joint_mode_ros.pb.h"
#include "m3/robots/chain_name.h"
#include "m3/hardware/smoothing_mode.pb.h"

#define NDOF_ARM 7

class HumanoidDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_pub_;

public:
  //! ROS node initialization
  HumanoidDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_pub_ = nh_.advertise<m3ctrl_msgs::M3JointCmd>("/humanoid_command", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    char cmd[50];
     m3ctrl_msgs::M3JointCmd humanoid_cmd;
     
    humanoid_cmd.chain.resize(NDOF_ARM);
    humanoid_cmd.stiffness.resize(NDOF_ARM);
    humanoid_cmd.position.resize(NDOF_ARM);
    humanoid_cmd.velocity.resize(NDOF_ARM);
    humanoid_cmd.effort.resize(NDOF_ARM);
    humanoid_cmd.control_mode.resize(NDOF_ARM);
    humanoid_cmd.smoothing_mode.resize(NDOF_ARM);
    humanoid_cmd.chain_idx.resize(NDOF_ARM);
     
    //  center robot first
    
    std::cout << "Gravity Compenstation Demo.\n";
    std::cout << "Commanding right arm in mode JOINT_MODE_THETA_GC.\n";
    std::cout << "Stiffness is 0.0.\n";    
    
    
    std::cout << "\nPress any key to Start.\n";
    
    std::cin.getline(cmd, 50);
    
    
    humanoid_cmd.header.stamp = ros::Time::now();
    humanoid_cmd.header.frame_id = "humanoid_cmd";
    
    double stiffness = 0.0;
    
    for (int i = 0; i < NDOF_ARM; i++)
    {
      humanoid_cmd.chain[i] = (unsigned char)RIGHT_ARM; // chain name: RIGHT_ARM, HEAD, RIGHT_HAND, LEFT_ARM, or LEFT_HAND    
      humanoid_cmd.chain_idx[i] = i;
      humanoid_cmd.control_mode[i] = (unsigned char)JOINT_MODE_ROS_THETA_GC; //Compliant position mode      
      humanoid_cmd.smoothing_mode[i] = (unsigned char)SMOOTHING_MODE_SLEW; //Smooth trajectory
      //humanoid_cmd.smoothing_mode[0] = (unsigned char)SMOOTHING_MODE_MIN_JERK; //Use for HEAD
      humanoid_cmd.velocity[i] = 1.0; //Rad/s
      humanoid_cmd.stiffness[i] = stiffness; //0-1.0
      humanoid_cmd.effort[i] = 0.0;
      humanoid_cmd.position[i] = 0.0; //Desired position (Rad)      
    }

    
    cmd_pub_.publish(humanoid_cmd);
      
    
    std::cout << "\nPress any key to exit.\n";
    
      std::cin.getline(cmd, 50);
      
      // Put all joints in mode OFF before quitting
     for (int i = 0; i < NDOF_ARM; i++)
    {      
      humanoid_cmd.control_mode[i] = (unsigned char)JOINT_MODE_ROS_OFF; //Compliant position mode            
    } 
    
      humanoid_cmd.header.stamp = ros::Time::now();
      //publish the assembled command
      cmd_pub_.publish(humanoid_cmd);
    
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  HumanoidDriver driver(nh);
  driver.driveKeyboard();
}
