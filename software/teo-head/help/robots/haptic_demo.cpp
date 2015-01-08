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

#include "m3/robots/haptic_demo.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"
using namespace std;
using namespace KDL;

namespace m3{
	
using namespace m3rt;
using namespace std;
	
		
///////////////////////////////////////////////////////


void M3HapticDemo::Startup()
{
	if (bot==NULL)
		SetStateError();
	else
		SetStateSafeOp();

}

void M3HapticDemo::Shutdown()
{
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						  
bool M3HapticDemo::ReadConfig(const char * filename)
{
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	if (!M3Component::ReadConfig(filename))
		return false;
	doc["humanoid"] >> bot_name;
	double val;
	doc["param"]["max_fx"] >> val;
	param.set_max_fx(val);
	doc["param"]["max_fy"] >> val;
	param.set_max_fy(val);
	doc["param"]["max_fz"] >> val;
	param.set_max_fz(val);
	
	doc["param"]["x_k"] >> val;
	param.set_x_k(val);
	doc["param"]["y_k"] >> val;
	param.set_y_k(val);
	doc["param"]["z_k"] >> val;
	param.set_z_k(val);
	
	doc["param"]["x_k_d"] >> val;
	param.set_x_k_d(val);
	doc["param"]["y_k_d"] >> val;
	param.set_y_k_d(val);
	doc["param"]["z_k_d"] >> val;
	param.set_z_k_d(val);
	
	doc["param"]["x_desired"] >> val;
	param.set_x_desired(val);
	doc["param"]["y_desired"] >> val;
	param.set_y_desired(val);
	doc["param"]["z_desired"] >> val;
	param.set_z_desired(val);
	
	x_df.ReadConfig(doc["calib"]["x_df"]);
 	z_df.ReadConfig(doc["calib"]["z_df"]);
	
	
	return true;
}

bool M3HapticDemo::LinkDependentComponents()
{
	//Need to find at least one arm
	bot=(M3Humanoid*) factory->GetComponent(bot_name);
	if (bot==NULL)
		M3_INFO("M3Humanoid component %s not found for component %s\n",bot_name.c_str(),GetName().c_str());
	if (bot==NULL)
		return false;
	return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3HapticDemo::StepStatus()
{
	status.set_foo(1.0);
}

void M3HapticDemo::StepCommand()
{
	int i;
	/*mReal x_desired = 0.24;
	mReal y_desired = 0.27;
	mReal z_desired = -0.48;*/
	
	mReal x_desired = param.x_desired();
	mReal y_desired = param.y_desired();
	mReal z_desired = param.z_desired();
	
	//mReal x_k = 10000.0;
	//mReal y_k = 10000.0;
	//mReal z_k = 10000.0;
		
	mReal x_k = param.x_k();
	mReal y_k = param.y_k();
	mReal z_k = param.z_k();
	
	//M3Arm * arm = bot->GetLeftArm();
	
	Eigen::Vector3d end_pos = bot->GetEndPosition(RIGHT_ARM);
	
	mReal x_err = (x_desired - end_pos[0]);
	mReal y_err = (y_desired - end_pos[1]);
	mReal z_err = (z_desired - end_pos[2]);
		
	
			
	//mReal fx = x_k * (x_err);
	mReal fy = y_k * (y_err);
	//mReal fz = z_k * (z_err);
	
	
	mReal x_dx = x_df.Step(end_pos[0]);
	mReal z_dx = z_df.Step(end_pos[2]);
	
	mReal fx =pid_x.Step(end_pos[0],
			      x_dx,
			      x_desired,
			      x_k,
			      0, // i
			      param.x_k_d(), // d
			      0, // i_limit
			      0); // i_range
			      
	mReal fz =pid_z.Step(end_pos[2],
			      z_dx,
			      z_desired,
			      z_k,
			      0, // i
			      param.z_k_d(), // d
			      0, // i_limit
			      0); // i_range
	
			      
	
	tmp_cnt++;
	if (command.enable())
	{
	  
	  /*if (tmp_cnt%100==0)
	     M3_INFO("Err: %f %f %f\n",x_err, y_err, z_err);*/
	  
	    /*fx = 0;
	    fy = 0;
	    fz = 0;*/
	    bot->SetMotorPowerOn();
	    Eigen::Matrix<double,6,1> wrench;
	    /*wrench[0]=CLAMP(command.fx(),-param.max_fx(),param.max_fx());
	    wrench[1]=CLAMP(command.fy(),-param.max_fy(),param.max_fy());
	    wrench[2]=CLAMP(command.fz(),-param.max_fz(),param.max_fz());*/
	    /*wrench[0]=CLAMP(fx,-param.max_fx(),param.max_fx());
	    wrench[1]=CLAMP(fy,-param.max_fy(),param.max_fy());
	    wrench[2]=CLAMP(fz,-param.max_fz(),param.max_fz());*/
	    wrench[0]=fx;
	    wrench[1]=fy;
	    wrench[2]=fz;
	    wrench[3]=0;
	    wrench[4]=0;
	    wrench[5]=0;
	    /*if (tmp_cnt%100==0)
	     M3_INFO("F: %f %f %f\n",fx, fy, fz);*/
	    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> J = bot->GetJacobian(RIGHT_ARM);
	    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> JT=J.transpose();
	    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> tq=JT*wrench;
	    
	      for (i=0;i<4;i++)
	      {
		if (i == 2)
		  {
		    bot->SetModeThetaGc(RIGHT_ARM,i);
		    bot->SetStiffness(RIGHT_ARM,i,0.7);
		    bot->SetThetaDeg(RIGHT_ARM,i,0.0);
		    bot->SetSlewRateProportional(RIGHT_ARM,i, 1.0);
		  }
		  else
		  {
		    bot->SetModeTorqueGc(RIGHT_ARM,i);
		    bot->SetStiffness(RIGHT_ARM,i,0.0);
		    //bot->SetModeThetaGc(RIGHT_ARM,i);
		    bot->SetSlewRateProportional(RIGHT_ARM,i, 1.0);
		    bot->SetTorque_mNm(RIGHT_ARM,i,tq[i]);
		    //bot->SetTorque_mNm(RIGHT_ARM,i,0);
		  }
		  /*if (tmp_cnt%100==0)
			M3_INFO("On: %d : %f\n",i,tq[i]);*/
		  
	      }
	      for (i=4;i<7;i++)
	      {
		  bot->SetModeThetaGc(RIGHT_ARM,i);
		  bot->SetStiffness(RIGHT_ARM,i,0.7);
		  if (i == 5)
		  {
		    mReal j5 = CLAMP(90.0 - bot->GetThetaDeg(RIGHT_ARM,3), -50.0, 50);
		    bot->SetThetaDeg(RIGHT_ARM,i,j5);
		  }
		  else
		  {
		    bot->SetThetaDeg(RIGHT_ARM,i,0.0);
		  }
		  bot->SetSlewRateProportional(RIGHT_ARM,i, 1.0);
		  //bot->SetTorque_mNm(RIGHT_ARM,i,tq[i]);
		  //bot->SetTorque_mNm(RIGHT_ARM,i,0);
		  /*if (tmp_cnt%100==0)
			M3_INFO("On: %d : %f\n",i,tq[i]);*/
	      }
	}
	/*else
	{
	  //if (tmp_cnt%100==0)
	   //   M3_INFO("Off\n");
	  bot->SetMotorPowerOff();
	  for (i=0;i<arm->GetNumDof();i++)
		  bot->SetCtrlModeTorqueOff(M3Humanoid::RIGHT_ARM,i);
		
	}*/
}

}