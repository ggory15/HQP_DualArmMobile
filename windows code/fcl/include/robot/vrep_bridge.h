#pragma once
#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>


using namespace std;

extern "C" {
#include "extApi.h"
}

const int MOTORNUM = 7; 	/// < Depends on simulation envrionment

class VRepBridge
{
public:
	VRepBridge();
	~VRepBridge();

	bool simConnectionCheck();
	void simLoop();
	void write();
	void read();

	enum VrepControlMode { TORQUE_CTRL, POSITION_CTRL };
	void setMode(VrepControlMode mode);

public:
	Eigen::VectorXd current_q_;
	Eigen::VectorXd current_q_dot_;
	Eigen::VectorXd desired_q_;
	Eigen::VectorXd desired_torque_;
	Eigen::VectorXd target_x_;


	simxFloat dist1_[3];
	simxFloat dist2_[3];
	simxFloat dist3_[3];	
	simxFloat dist4_[3];	
	simxFloat dist5_[3];	
	simxFloat dist6_[3];	

	simxFloat target_pos[3];

	const size_t getTick() { return tick_; }

private:
	VrepControlMode mode_;
private:
	simxInt clientID_;
	simxInt motorHandle_[MOTORNUM];	/// < Depends on simulation envrionment
	simxInt objectHandle_;
	simxInt distHandle[4];
	
	simxInt targetHandle_;
	size_t tick_;

	void simxErrorCheck(simxInt error);
	void dataInit();
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment
};
