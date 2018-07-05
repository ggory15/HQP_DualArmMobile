#include <iostream>
#include "vrep_bridge.h"

//for robot model
#include "robot_model.h"
//for controller 
#include "Inverse-dynamics.h"

// for tasks
#include "task-com.h"
#include "task-operational.h"
#include "task-joint-posture.h"
#include "task-joint-bounds.h"

// for trajectories 
#include "trajectory-operationalspace.h"
#include "trajectory-jointspace.h"

// for solver
#include "solver-HQP-factory.hxx"
#include "solver-utils.h"
//#include <tsid/math/utils.hpp>

#include <conio.h> // for keyboard hit
using namespace std;

HQP::robot::RobotModel * robot_;
HQP::InverseDynamics * invdyn_;
HQP::tasks::TaskJointPosture * postureTask;
HQP::tasks::TaskOperationalSpace * moveTask;
HQP::tasks::TaskJointLimit * jointLimitTask;
VectorXd q(dof);
VectorXd qdot(dof);
VectorXd qdes(dof);
VectorXd q_lb(dof);
VectorXd q_ub(dof);
double time = 0.0;
double Hz = 1000.0;


using namespace HQP;
int main()
{
	// for robot
	robot_ = new HQP::robot::RobotModel(HQP::robot::Type::Manipulator);
	invdyn_ = new HQP::InverseDynamics(*robot_);
	q.setZero();
	qdot.setZero();
	invdyn_->computeProblemData(time, q, qdot);
	solver::SolverHQPBase * solver = solver::SolverHQPFactory::createNewSolver(solver::SOLVER_HQP_EIQUADPROG, "solver-qpoases");
	solver->resize(invdyn_->nVar(), invdyn_->nEq(), invdyn_->nIn(), invdyn_->nBound()); 

	qdes = 30.0/180.0*3.14*VectorXd(7).setOnes();
	q_lb = -50.0 / 180.0 * M_PI * VectorXd(dof).setOnes();
	q_ub = -1.0*q_lb;
	//q_ub(0) = 15.0 / 180.0*M_PI;

	jointLimitTask = new tasks::TaskJointLimit("joint_limit_task", *robot_);
	double kp_jointlimit = 30.0, w_jointlimit = 1.00;
	jointLimitTask->Kp(kp_jointlimit*VectorXd::Ones(robot_->nq()));
	jointLimitTask->Kd(2.0*jointLimitTask->Kp().cwiseSqrt());
	jointLimitTask->setJointLimit(q_lb, q_ub);
	invdyn_->addJointLimitTask(*jointLimitTask, w_jointlimit, 0, 0.0);

	postureTask = new tasks::TaskJointPosture("joint_control_task", *robot_);
	double kp_posture = 30.0, w_posture = 1.00;
	postureTask->Kp(kp_posture*VectorXd::Ones(robot_->nq()));
	postureTask->Kd(2.0*postureTask->Kp().cwiseSqrt());
	invdyn_->addJointPostureTask(*postureTask, w_posture, 2, 0.0);

	moveTask = new tasks::TaskOperationalSpace("end_effector_task", *robot_, 7);
	double kp_move = 100.0, w_move = 1.0;
	moveTask->Kp(kp_move*VectorXd::Ones(6));
	moveTask->Kd(2.0*moveTask->Kp().cwiseSqrt());

	trajectories::TrajectoryBase *trajPosture = new trajectories::TrajectoryJointConstant("joint_traj", qdes);
	
	trajectories::TrajectorySample samplePosture(robot_->nv());
	
	Transform3d T_endeffector;
	trajectories::TrajectorySample s(12, 6);
	trajectories::TrajectoryOperationConstant *trajEE = new trajectories::TrajectoryOperationConstant("operational_traj", T_endeffector);
	solver->resize(invdyn_->nVar(), invdyn_->nEq(), invdyn_->nIn(), invdyn_->nBound());

	// for v-rep
	VRepBridge vb;
	vb.isSimulationRun = false;
	vb.exitFlag = false;

	while (vb.simConnectionCheck() && !vb.exitFlag)
	{
		if (_kbhit()) {
			int key = _getch();
			switch (key)
			{
			case '\t':
				if (vb.isSimulationRun) {
					cout << "Simulation Pause" << endl;
					vb.isSimulationRun = false;
				}
				else {
					cout << "Simulation Run" << endl;
					vb._cntt = 0;
					vb.isSimulationRun = true;
				}
				break;

			case 'q':
				vb.isSimulationRun = false;
				vb.exitFlag = true;
				simxEndDialog(vb.getClientID(), vb.dialog_handle, simx_opmode_oneshot);
				break;

			default:
				break;
			}
		}
		if (vb.isSimulationRun)
		{
			vb.read();
			if (vb._cntt > 0)
			{
				time = vb._cntt / Hz;
				robot_->getUpdateKinematics(vb.current_q_, vb.current_qdot_);
				if (time == 1.0 / Hz) {
					T_endeffector = robot_->getTransformation(7);
					T_endeffector.translation()(2) -= 0.1;
					
					trajEE->setReference(T_endeffector);
					s = trajEE->computeNext();

					moveTask->setReference(s);
					bool sucess = invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.1);	
				}
					s = trajEE->computeNext();
					moveTask->setReference(s);

				const solver::HQPData & HQPData = invdyn_->computeProblemData(time, vb.current_q_, vb.current_qdot_);

				if (time == 1 / Hz)
					cout << solver::HQPDataToString(HQPData, true) << endl;


				samplePosture = trajPosture->computeNext();
				postureTask->setReference(samplePosture);

				const solver::HQPOutput & sol = solver->solve(HQPData);
				
				const VectorXd & tau = invdyn_->getActuatorForces(sol);
				const VectorXd & dv = invdyn_->getAccelerations(sol);

				vb.desired_torque_ = tau;

				vb.write();
			}
		
			vb.simLoop();
			vb._cntt++;

		}
	}

	return 0;
}