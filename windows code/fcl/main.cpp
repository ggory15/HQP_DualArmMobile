#include <iostream>
#include <string>
#include "include/robot/vrep_bridge.h"
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include "include/robot/controller.h"
#include "include/fcl/fcl_model.h"

using namespace std;
 
bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}


int main()
{
	VRepBridge vb;

	FCL_MODEL fm;
	fm.Initialize_Model();

	

	const double hz = 1000;
	ArmController ac(hz);
	bool is_simulation_run = false;
	bool exit_flag = false;
	bool is_first = true;
	bool is_position = false;
	//vb.setMode(VRepBridge::POSITION_CTRL);
	vb.setMode(VRepBridge::TORQUE_CTRL);
	ac.setMode(ArmController::GRAVITY_COMP);

	cout << "Torque mode is ON" << endl;
	cout << " Please use torque control 't' key" << endl;

	while (vb.simConnectionCheck() && !exit_flag)
	{
		if (is_first)
		{
			vb.read();
			vb.current_q_.setZero();
			vb.current_q_dot_.setZero();
			cout << vb.current_q_ << endl;
			is_first = false;
			ac.readData(vb.current_q_, vb.current_q_dot_);

			//ac.initPosition();
		}
		else
		{
			vb.read();
			ac.readData(vb.current_q_, vb.current_q_dot_);
		}
		if (_kbhit())
		{
			int key;
			key = getchar();
			switch (key)
			{
				// Implement with user input
			case 'i':
				cout << "Joint control to initial position" << endl;
				ac.setMode(ArmController::INIT_JOINT_CTRL);
				vb.setMode(VRepBridge::TORQUE_CTRL);
				is_position = false;
				break;
			case 'h':
				cout << "Joint control to home position" << endl;
				ac.setMode(ArmController::HOME_JOINT_CTRL);
				is_position = true;
				break;
			case 'l':
				ac.setMode(ArmController::SCA_Weighting);
				vb.setMode(VRepBridge::TORQUE_CTRL);
				is_position = false;	
				break;
			case 'a':
				ac.setMode(ArmController::SCA_Task_Transition);
				vb.setMode(VRepBridge::TORQUE_CTRL);
				is_position = false;	
				break;
			case 'b':
				ac.setMode(ArmController::SCA_Adding_Two_Tasks);
				vb.setMode(VRepBridge::TORQUE_CTRL);
				is_position = false;
				break;				
			case '\t':
				if (is_simulation_run) {
					cout << "Simulation Pause" << endl;
					is_simulation_run = false;
				}
				else {
					cout << "Simulation Run" << endl;
					is_simulation_run = true;
				}
				break;
			case 'q':
				is_simulation_run = false;
				exit_flag = true;
				break;
			default:
				break;
			}
		}

		if (is_simulation_run) {
			ac.compute();
				for(int i=0;i<7;i++)
				{	
					for(int j=0;j<3;j++)
							for(int k=0;k<3;k++)
							{
							fm.Rot[i](j,k) = ac.link_rot[i](j,k);
							fm.temp[k]= ac.link_com[i](k);
							}
				fm.Trs[i].setValue(fm.temp[0], fm.temp[1], fm.temp[2]);	
				}
						
			fm.Check_Collision_Pairs();

			for (int i=0;i<4;i++)
			{
			ac.dis[i] = fm.Dis[i].distance;
				for(int j=0;j<3;j++)
				{
				ac.P_b[i](j) = fm.Dis[i].p1[j];
				ac.P_a[i](j) = fm.Dis[i].p2[j];
				}
			ac.dist_vector(i) = ac.dis[i];
			}
			int min_coef;
			ac.dist_vector.minCoeff(&min_coef);

			for (int i=0;i<3;i++)
			{
			vb.dist1_[i] = fm.Dis[min_coef].p2[i];
			vb.dist2_[i] = fm.Dis[min_coef].p1[i];
//			vb.dist3_[i] = fm.Dis[2].p2[i];
//			vb.dist4_[i] = fm.Dis[3].p2[i];
			vb.target_pos[i] = ac.target_pos(i);
			}


			if (is_position)
			{
				ac.writePositionData(vb.desired_q_);
			}
			else
			{
				ac.writeTorqueData(vb.desired_torque_);
			}


			vb.write();
			vb.simLoop();
		}
	}

	return 0;
}
