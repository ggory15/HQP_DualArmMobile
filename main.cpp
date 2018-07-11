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
#include "task-mobile.h"
#include "task-singularity.h"

// for trajectories 
#include "trajectory-operationalspace.h"
#include "trajectory-jointspace.h"

// for solver
#include "solver-HQP-factory.hxx"
#include "solver-utils.h"
//#include <tsid/math/utils.hpp>

// for contact point

#include "contact-3d.h"

#include "container.h"
#include <string>
#include <vector>
#include <conio.h> // for keyboard hit

// for time check
#define timer
#ifdef timer
#include <windows.h>
	double PCFreq = 0.0;
	__int64 CounterStart = 0;

	void StartCounter()
	{
		LARGE_INTEGER li;
		if (!QueryPerformanceFrequency(&li))
			cout << "QueryPerformanceFrequency failed!\n";

		PCFreq = double(li.QuadPart) / 1000.0;

		QueryPerformanceCounter(&li);
		CounterStart = li.QuadPart;
	}
	double GetCounter()
	{
		LARGE_INTEGER li;
		QueryPerformanceCounter(&li);
		return double(li.QuadPart - CounterStart) / PCFreq;
	}
#endif // timer
using namespace std;

HQP::robot::RobotModel * robot_;
HQP::InverseDynamics * invdyn_;
HQP::tasks::TaskJointPosture * postureTask;
HQP::tasks::TaskOperationalSpace * moveTask, * move2Task;
HQP::tasks::TaskMobile * mobileTask;
HQP::tasks::TaskJointLimit * jointLimitTask;
HQP::contact::Contact3dPoint * contactTask;
HQP::tasks::TaskSingularityAvoidance * singularTask;

VectorXd q(dof);
VectorXd qdot(dof);
VectorXd qdes(dof);
VectorXd q_lb(dof + 2); // mobile 2 + robot 7
VectorXd q_ub(dof + 2); // mobile 2 + robot 7
double time = 0.0;
double Hz = 1000.0;
int na;
int nv;
int nq;


using namespace HQP;
int main()
{
	// for robot
	robot_ = new HQP::robot::RobotModel(HQP::robot::Type::MobileManipulator);
	na = robot_->na();
	nv = robot_->nv();

	
	invdyn_ = new HQP::InverseDynamics(*robot_);
	q.setZero();
	qdot.setZero();	

	solver::SolverHQPBase * solver = solver::SolverHQPFactory::createNewSolver(solver::SOLVER_HQP_EIQUADPROG, "solver-eiquadprog");	
	
	// Level 0 : Joint Velocity Limit for Mobile + Manipulator 
	q_lb = -180.0 / 180.0 * M_PI * VectorXd(dof + 2).setOnes();
	q_ub = -1.0*q_lb;
	//q_ub(5) = 45.0 * M_PI / 180.0;
	q_lb.head(2) = -30.0 * VectorXd(2).setOnes();
	q_ub.head(2) = 30.0 * VectorXd(2).setOnes();

	double kp_jointlimit = 30.0, w_jointlimit = 1.00;
	
	singularTask = new tasks::TaskSingularityAvoidance("sigunalr", *robot_);
	singularTask->Kp(kp_jointlimit * VectorXd::Ones(1));
	singularTask->Kd(2.0*singularTask->Kp().cwiseSqrt());
	
	//invdyn_->addSingularityTask(*singularTask, 1.0, 1, 0.0);
	
	jointLimitTask = new tasks::TaskJointLimit("joint_limit_task", *robot_);
	jointLimitTask->Kp(kp_jointlimit*VectorXd::Ones(robot_->nv()));
	jointLimitTask->Kd(2.0*jointLimitTask->Kp().cwiseSqrt());
	jointLimitTask->setJointLimit(q_lb, q_ub);

	invdyn_->addJointLimitTask(*jointLimitTask, w_jointlimit, 0, 0.0);



	qdes = 20.0 / 180.0 * 3.14*VectorXd(dof).setOnes();
	qdes.setZero();
	//qdes(1) = 30.0 * M_PI / 180.0;
	qdes(3) = -70.0 * M_PI/180.0;
	qdes(10) = -70.0 * M_PI / 180.0;
	//qdes(3) = 90.0 * M_PI / 180.0;
	//qdes(5) = -90.0 * M_PI / 180.0;
	
	postureTask = new tasks::TaskJointPosture("joint_control_task", *robot_);
	double kp_posture = 100.0, w_posture = 1.00;
	postureTask->Kp(kp_posture*VectorXd::Ones(robot_->nv()-2));
	postureTask->Kd(2.0*postureTask->Kp().cwiseSqrt());
	invdyn_->addJointPostureTask(*postureTask, 1.0, 2, 0.0);

	moveTask = new tasks::TaskOperationalSpace("end_effector_task", *robot_, 7);
	double kp_move = 300.0, w_move = 1.0;
	VectorXd a = VectorXd::Ones(6);
	a.tail(3) *= 10.0;
	moveTask->Kp(kp_move*a);
	moveTask->Kd(2.0*moveTask->Kp().cwiseSqrt());
	moveTask->setSingular(false);

	move2Task = new tasks::TaskOperationalSpace("end_effector_task2", *robot_, 7);
	move2Task->Kp(kp_move*a);
	move2Task->Kd(2.0*move2Task->Kp().cwiseSqrt());
	move2Task->setSingular(true);

	mobileTask = new tasks::TaskMobile("mobile_task", *robot_);
	double kp_mobile = 1000.0, w_mobile = 1.0; // 400 -> 9 deg 
	mobileTask->Kp(kp_mobile*VectorXd::Ones(6));
	mobileTask->Kd(2.0*mobileTask->Kp().cwiseSqrt());
	mobileTask->setOnlyOriCTRL(true);



	trajectories::TrajectoryBase *trajPosture = new trajectories::TrajectoryJointConstant("joint_traj", qdes);
	trajectories::TrajectorySample samplePosture(robot_->nv()-2);
	//
	Transform3d T_endeffector;
	trajectories::TrajectorySample s(12, 6);
	trajectories::TrajectoryOperationConstant *trajEE = new trajectories::TrajectoryOperationConstant("operational_traj", T_endeffector);
	//
	Transform3d T_mobile;
	trajectories::TrajectorySample s_mobile(12, 6);
	trajectories::TrajectoryOperationConstant *trajmobile = new trajectories::TrajectoryOperationConstant("mobile_traj", T_mobile);
	
	// QR Test //
	/*
	MatrixXd A(6, 14);
	A.setRandom();
	MatrixXd U, T;
	MatrixXd Z;

	StartCounter();
	for (int i = 0; i < 1000; i++) {
		//A.setRandom();
		Eigen::JacobiSVD< MatrixXd > svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
		U = svd.matrixU();
		T = svd.matrixV();
		Z = svd.singularValues();
	}
	cout << "svd" << "\t" << GetCounter() / 1000.0 << endl;

	StartCounter();
	for (int i = 0; i < 1000; i++) {
		//A.setRandom();
		Eigen::ColPivHouseholderQR<MatrixXd> qr(A);
		//Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A);
		U = qr.matrixQ();
		Z = qr.matrixR().topLeftCorner(6, 14).triangularView<Upper>();
		T = qr.colsPermutation();
	}
	cout << "cod" << "\t" << GetCounter() / 1000.0 << endl;
	cout << "A" << A << endl;
	cout << "Z" << Z << endl;
	cout << "Q" << U << endl;
	cout << "T" << T << endl;
	cout << "Z*Q" << U * Z * T<< endl;
	getchar();
	*/
	/*contactTask = new contact::Contact3dPoint("contact_end_ee", *robot_, 7, Eigen::Vector3d::UnitX(), 0.2, 0.5 , 20);
	contactTask->Kp(100.0*VectorXd::Ones(6));
	contactTask->Kd(2.0*contactTask->Kp().cwiseSqrt());
	Transform3d H_ee_ref;
	contactTask->setReference(H_ee_ref);
	invdyn_->addRigidContact(*contactTask, 1);
	*/
	
	solver->resize(invdyn_->nVar(), invdyn_->nEq(), invdyn_->nIn(), invdyn_->nBound());
	// for v-rep
	VRepBridge vb;
	vb.isSimulationRun = false;
	vb.exitFlag = false;

	bool flag = false;
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
#ifdef timer
				StartCounter();
#endif
				time = vb._cntt / Hz;

				VectorXd q_current(robot_->na() + 5), qdot_current(robot_->na() + 5);			
				
				q_current.setZero(); 
				q_current.head<2>() = vb.H_transform_.translation().head(2);
				q_current(2) = vb.euler_(2);
				q_current(3) = 0.0;
				q_current(4) = 0.0;			
				q_current.tail<dof>() = vb.current_q_;

				qdot_current.setZero();
				qdot_current.head<2>() = vb.H_vel_.linear().head(2);
				qdot_current(2) = vb.H_vel_.angular()(2);
				qdot_current(3) = vb.current_base_vel_(0);
				qdot_current(4) = vb.current_base_vel_(1);
				qdot_current.tail(dof) = vb.current_qdot_;
				
				robot_->getUpdateKinematics(q_current, qdot_current);

			//	cout << "miani double" << robot_->getManipulability(0, q_current) << endl;
				
				if (time == 1.0 / Hz) {
					T_endeffector = robot_->getTransformation(7);
					T_endeffector.translation()(0) += 0.33;
					//T_endeffector.translation()(1) -= 0.13;
		
					trajEE->setReference(T_endeffector);
					s = trajEE->computeNext();
					moveTask->setReference(s);
					move2Task->setReference(s);

					//bool sucess = invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);	
					
					invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);
					invdyn_->addOperationalTask(*move2Task, w_move, 1, 0.0);
					
					T_mobile = robot_->getMobileTransformation();
					trajmobile->setReference(T_mobile);
					s_mobile = trajmobile->computeNext();
					invdyn_->addMotionTask(*mobileTask, w_mobile, 2, 0.0);
					mobileTask->setReference(s_mobile);
					//
				}
				if (time == 0.5 ) {
				//	invdyn_->removeTask("end_effector_task");
					cout << "remove task" << endl;
				}
				

			/*	if (time == 1.0)
					invdyn_->addMotionTask(*mobileTask, w_mobile, 2, 0.0);*/
				MatrixXd J_EE_left(6, 7);
				J_EE_left = robot_->getJacobian(7).block(0, 2, 6, 7);

				Eigen::ColPivHouseholderQR<MatrixXd> qr(J_EE_left);
				MatrixXd U, Z, T;

				U = qr.matrixQ();
				//if (qr.matrixR().topLeftCorner(6, 7).triangularView<Upper>()(5, 5) < 0.05 && !flag) {
				//	invdyn_->removeTask("end_effector_task");
				//	invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);
				//	invdyn_->addOperationalTask(*move2Task, w_move, 1, 0.0);
				//	flag = true;
				//}
				//else {
				//	if (invdyn_->checkTask("end_effector_task2"))
				//		invdyn_->removeTask("end_effector_task2");
				//}
				//if (qr.matrixR().topLeftCorner(6, 7).triangularView<Upper>()(5, 5) < 0.05) {
				//	cout << "singular" << endl;
				//}
				//else {
				//	cout << "non singular" << endl;
				//}

			//	
				s = trajEE->computeNext();
				moveTask->setReference(s);
				move2Task->setReference(s);
				s_mobile = trajmobile->computeNext();
				mobileTask->setReference(s_mobile);

				samplePosture = trajPosture->computeNext();
				postureTask->setReference(samplePosture);
				const solver::HQPData & HQPData = invdyn_->computeProblemData(time, q_current, qdot_current);
				if (time == 1/Hz)
					cout << solver::HQPDataToString(HQPData, true) << endl;
				

			//	if (time == 1) {
			//		invdyn_->removeTask("end_effector_task");

			//		invdyn_->removeTask("mobile_task");
			//		T_endeffector = robot_->getTransformation(7);
			//		T_endeffector.translation()(2) -= 0.13;

			//		trajEE->setReference(T_endeffector);
			//		s = trajEE->computeNext();
			//		moveTask->setReference(s);

			//		bool sucess = invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);
			//	}

		
			////	cout << "sovler start" << endl;
				const solver::HQPOutput & sol = solver->solve(HQPData);
				const VectorXd & tau = invdyn_->getActuatorForces(sol);
				VectorXd dv = invdyn_->getAccelerations(sol);

			//	//cout << "contact force " << invdyn_->getContactForces(sol).transpose() << endl;

				vb.desired_base_vel_(0) = dv(0);
				vb.desired_base_vel_(1) = dv(1);
				vb.desired_base_vel_(2) = dv(1);
				vb.desired_base_vel_(3) = dv(0);


			//	
				vb.desired_torque_ = tau;
#ifdef timer
				cout << "svd" << "\t" << GetCounter() / 1000.0 << endl;
#endif
				vb.write();

			}
		
			vb.simLoop();
			vb._cntt++;

		}
	}

	return 0;
}