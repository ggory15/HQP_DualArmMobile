#include "include/robot/vrep_bridge.h"


VRepBridge::VRepBridge() : tick_(0)
{
	dataInit();
	simInit();
	getHandle();
}
VRepBridge::~VRepBridge()
{
	simxStopSimulation(clientID_, simx_opmode_oneshot_wait);
	simxFinish(clientID_);
}

bool VRepBridge::simConnectionCheck()
{
	return (simxGetConnectionId(clientID_) != -1);
}
void VRepBridge::simLoop()
{
	tick_++;
	simxSynchronousTrigger(clientID_);
}

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
	current_q_dot_.resize(MOTORNUM);
	desired_q_.resize(MOTORNUM);
	desired_torque_.resize(MOTORNUM);
	target_x_.resize(3);
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
	switch (mode_)
	{
	case TORQUE_CTRL:
	{
		for (size_t i = 0; i < MOTORNUM; i++)
		{
			simxFloat velocityLimit;

			if (desired_torque_(i) >= 0.0)
				velocityLimit = 10e10f;
			else
				velocityLimit = -10e10f;

			simxSetJointTargetVelocity(clientID_, motorHandle_[i], velocityLimit, simx_opmode_streaming);
			simxSetJointForce(clientID_, motorHandle_[i], static_cast<float>(abs(desired_torque_(i))), simx_opmode_streaming);

		}
		break;
	}
	case POSITION_CTRL:
	{
		for (size_t i = 0; i < MOTORNUM; i++)
		{
			simxSetJointTargetPosition(clientID_, motorHandle_[i], desired_q_(i), simx_opmode_streaming);
		}
		break;
	}
	}

	
	simxSetObjectPosition(clientID_, distHandle[0], -1, dist1_ ,simx_opmode_streaming);
	simxSetObjectPosition(clientID_, distHandle[1], -1, dist2_ ,simx_opmode_streaming);
	simxSetObjectPosition(clientID_, distHandle[2], -1, dist3_ ,simx_opmode_streaming);
	simxSetObjectPosition(clientID_, distHandle[3], -1, dist4_ ,simx_opmode_streaming);

	simxSetObjectPosition(clientID_, targetHandle_, -1, target_pos ,simx_opmode_streaming);

}
void VRepBridge::read()
{
	for (size_t i = 0; i < MOTORNUM; i++)
	{
		simxFloat data;
		simxGetJointPosition(clientID_, motorHandle_[i], &data, simx_opmode_streaming);
		current_q_(i) = data;
		simxGetObjectFloatParameter(clientID_, motorHandle_[i], 2012, &data, simx_opmode_streaming);
		current_q_dot_(i) = data;
	}
}
void VRepBridge::setMode(VrepControlMode mode)
{
	mode_ = mode;
}

void VRepBridge::getHandle()
{
	cout << "[INFO] Getting handles." << endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "rot1", &motorHandle_[0], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "rot2", &motorHandle_[1], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "rot3", &motorHandle_[2], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "rot4", &motorHandle_[3], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "rot5", &motorHandle_[4], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "rot6", &motorHandle_[5], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "rot7", &motorHandle_[6], simx_opmode_oneshot_wait));

	simxErrorCheck(simxGetObjectHandle(clientID_, "Sphere", &objectHandle_, simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "Sphere0", &distHandle[0], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "Sphere1", &distHandle[1], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "Sphere2", &distHandle[2], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "Sphere3", &distHandle[3], simx_opmode_oneshot_wait));

	simxErrorCheck(simxGetObjectHandle(clientID_, "Sphere4", &targetHandle_, simx_opmode_oneshot_wait));


	cout << "[INFO] The handle has been imported." << endl;
}
