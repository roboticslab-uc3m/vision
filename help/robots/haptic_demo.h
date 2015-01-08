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


#ifndef M3_HAPTIC_DEMO_H
#define M3_HAPTIC_DEMO_H

#include "m3rt/base/component.h"
#include <google/protobuf/message.h>
#include "m3/robots/haptic_demo.pb.h"
#include "m3/chains/arm.h"
#include "m3/robots/humanoid.h"

namespace m3
{
	using namespace std;	
	using namespace KDL;
//Example component class. Sums the current consumption of both arms and scales it by param.scalar
class M3HapticDemo : public m3rt::M3Component
{
	public:
		M3HapticDemo(): m3rt::M3Component(ROBOT_CTRL_PRIORITY),bot(NULL){RegisterVersion("default",DEFAULT);}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	public:
	protected:
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
	protected:
		M3HapticDemoStatus status;
		M3HapticDemoCommand command;
		M3HapticDemoParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
	private:
		enum {DEFAULT};
		string bot_name;
		M3Humanoid * bot;
		int tmp_cnt;
		M3PID pid_x;		
		M3PID pid_z;
		M3DFilter x_df;
		M3DFilter z_df;
};


}

#endif


