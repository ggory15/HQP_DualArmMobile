#include "vrep_bridge.h"
using namespace std;

void VRepBridge::simxErrorCheck(simxInt error)
{
	string errorMsg;
	switch (error)
	{
	case simx_error_noerror:
		return;	// no error
		break;
	case simx_error_timeout_flag:
		errorMsg = "The function timed out (probably the network is down or too slow)";
		break;
	case simx_error_illegal_opmode_flag:
		errorMsg = "The specified operation mode is not supported for the given function";
		break;
	case simx_error_remote_error_flag:
		errorMsg = "The function caused an error on the server side (e.g. an invalid handle was specified)";
		break;
	case simx_error_split_progress_flag:
		errorMsg = "The communication thread is still processing previous split command of the same type";
		break;
	case simx_error_local_error_flag:
		errorMsg = "The function caused an error on the client side";
		break;
	case simx_error_initialize_error_flag:
		errorMsg = "simxStart was not yet called";
		break;
	default:
		errorMsg = "Unknown error.";
		break;
	}

	cout << "[ERROR] An error is occured. code = " << error << endl;
	cout << " - Description" << endl;
	cout << " | " << errorMsg << endl;

	throw std::string(errorMsg);
}

void VRepBridge::dataInit()
{
	current_q_.resize(MOTORNUM);
	current_q_.setZero();
	current_qdot_.resize(MOTORNUM);
	current_qdot_.setZero();	
	desired_q_.resize(MOTORNUM);
	desired_q_.setZero();	
	target_x_.resize(3);
	target_x_.setZero();	
	desired_torque_.resize(MOTORNUM);
	desired_torque_.setZero();

	force_.setZero();
	desired_obs_pos.setZero();
	target_pos.setZero();

	_cntt = 0;

}

void VRepBridge::simInit()
{
	simxFinish(-1);
	clientID_ = simxStart("127.0.0.1", 19997, true, true, 2000, 5);
	if (clientID_ < 0)
	{
		throw std::string("Failed connecting to remote API server. Exiting.");
	}


	simxErrorCheck(simxStartSimulation(clientID_, simx_opmode_oneshot_wait));
	simxErrorCheck(simxSynchronous(clientID_, true));

	cout << "[INFO] V-Rep connection is established." << endl;

}

void VRepBridge::write()
{
	for (size_t i = 0; i < MOTORNUM; i++) {
		simxFloat velLimit;

		if (desired_torque_(i) >= 0.0)
			velLimit = 10e10f;
		else
			velLimit = -10e10f;

		simxSetJointTargetVelocity(clientID_, motorHandle_[i], velLimit, simx_opmode_streaming);
		simxSetJointForce(clientID_, motorHandle_[i], static_cast<float>(abs(desired_torque_(i))), simx_opmode_streaming);
	}
}
void VRepBridge::read()
{
	for (size_t i = 0; i < MOTORNUM; i++)
	{
		simxFloat data;
		simxGetJointPosition(clientID_, motorHandle_[i], &data, simx_opmode_streaming);
		current_q_(i) = data;

		simxGetObjectFloatParameter(clientID_, motorHandle_[i], 2012, &data, simx_opmode_streaming);
		current_qdot_(i) = data;
	}
}


void VRepBridge::getHandle()
{
	cout << "[INFO] Getting handles." << endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint1", &motorHandle_[0], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint2", &motorHandle_[1], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint3", &motorHandle_[2], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint4", &motorHandle_[3], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint5", &motorHandle_[4], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint6", &motorHandle_[5], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "panda_joint7", &motorHandle_[6], simx_opmode_oneshot_wait));

	cout << "[INFO] The handle has been imported." << endl;
}
