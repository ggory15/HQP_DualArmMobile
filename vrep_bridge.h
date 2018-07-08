#pragma once
#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>
#include "motion.h"
using namespace std;

extern "C" {
#include "extApi.h"
}

const int MOTORNUM = 7;	/// < Depends on simulation envrionment

class VRepBridge
{
private:
	typedef std::function<void()> callfunc; // loop callback function

public:
	VRepBridge() : tick_(0)
	{
		dataInit();
		simInit();
		getHandle();
	}
	~VRepBridge()
	{
		simxStopSimulation(clientID_, simx_opmode_oneshot_wait);
		simxFinish(clientID_);
	}

	bool simConnectionCheck()
	{
		return (simxGetConnectionId(clientID_) != -1);
	}
	void simLoop()
	{
		//loopCallbackFunc();
		tick_++;
		simxSynchronousTrigger(clientID_);
	}
	void write();
	void read();

	const simxInt getClientID() {
		return clientID_;
	}


public:
	Eigen::VectorXd current_q_;
	Eigen::VectorXd current_qdot_;
	Eigen::VectorXd desired_q_;
	Eigen::VectorXd desired_torque_;
	Eigen::VectorXd target_x_;

	Eigen::Vector3d force_;
	Eigen::Vector3d torque_;
	Eigen::Vector3d desired_obs_pos;
	Eigen::Vector3d target_pos;
	Eigen::Vector3d euler_;
	Eigen::VectorXd desired_base_vel_, current_base_vel_;

	Transform3d H_transform_;
	HQP::MotionVector<double> H_vel_;

	const size_t getTick() { return tick_; }


	bool isSimulationRun = false;
	bool exitFlag = false;

	double Hz;
	int _cntt;

	simxInt dialog_handle;

private:
	simxInt clientID_;
	simxInt motorHandle_[MOTORNUM];	/// < Depends on simulation envrionment
	simxInt baseHandle_[4];
	simxInt objectHandle_;
	simxInt StateEstimator_;
	simxInt Obstacle;
	simxInt Target;
	simxInt Collection_;

	size_t tick_;
	//callfunc loopCallbackFunc;

	void simxErrorCheck(simxInt error);
	void dataInit();
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment
};
